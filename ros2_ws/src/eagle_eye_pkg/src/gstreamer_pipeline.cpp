#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include "eagle_eye_interfaces/msg/face_detect.hpp"
#include "eagle_eye_interfaces/msg/camera_orientation.hpp"

#include "eagle_eye_pkg/cameradata.h"
#include "eagle_eye_pkg/gstinferencemeta.h"

class GStreamerPipeline : public rclcpp::Node {
public:
    GStreamerPipeline()
        : Node("gstreamer_pipeline") {

	// define node input parameters
	std::string camera_url_ = "";
	this->declare_parameter<std::string>("camera_url", "rtsp://192.168.1.11:554/stream1");
	this->get_parameter("camera_url", camera_url_);

        face_detect_publisher_ = this->create_publisher<eagle_eye_interfaces::msg::FaceDetect>("face_detect", 10);

        inclinometer_subscription_ = this->create_subscription<eagle_eye_interfaces::msg::CameraOrientation>(
		"camera_orientation", 10, 
		std::bind(&GStreamerPipeline::inclinometer_callback, this, std::placeholders::_1)
        );

        // Initialize GStreamer
        gst_init(nullptr, nullptr);

        // Build the pipeline string
	std::string pipeline_str = "rtspsrc location=" + camera_url_ + " ! "
                                   "rtph265depay ! h265parse ! omxh265dec ! videoconvert ! "
                                   "video/x-raw,format=NV12 ! tee name=t ! queue ! "
                                   "vvas_xmultisrc kconfig=\"/opt/xilinx/kr260-eagle-eye/share/vvas/facedetect/preprocess.json\" ! queue ! "
                                   "vvas_xfilter kernels-config=\"/opt/xilinx/kr260-eagle-eye/share/vvas/facedetect/aiinference.json\" ! "
                                   "ima.sink_master vvas_xmetaaffixer name=ima ima.src_master ! fakesink "
                                   "t. ! queue max-size-buffers=1 leaky=2 ! ima.sink_slave_0 ima.src_slave_0 ! queue ! "
                                   "vvas_xfilter name=draw kernels-config=\"/opt/xilinx/kr260-eagle-eye/share/vvas/facedetect/drawresult.json\" ! queue ! "
                                   "kmssink driver-name=xlnx plane-id=39 sync=false fullscreen-overlay=true";

	// Convert the pipeline string to const gchar*
    	const gchar *pipeline_cstr = pipeline_str.c_str();

    	// Create the pipeline
    	pipeline_ = gst_parse_launch(pipeline_cstr, nullptr);
        if (!pipeline_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline");
            rclcpp::shutdown();
            return;
        }

        // Get the ima vvas_xmetaaffixer element
        ima_probe_ = gst_bin_get_by_name(GST_BIN(pipeline_), "ima");
        if (!ima_probe_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get ima element");
            gst_object_unref(pipeline_);
            rclcpp::shutdown();
            return;
        }

        // Attach a probe to the src pad of the element
        GstPad *ima_pad = gst_element_get_static_pad(ima_probe_, "src_master");
        if (!ima_pad) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get src pad from ima element");
            gst_object_unref(ima_probe_);
            gst_object_unref(pipeline_);
            rclcpp::shutdown();
            return;
        }

	// Attach callback to the probe
        gst_pad_add_probe(ima_pad, GST_PAD_PROBE_TYPE_BUFFER, ima_probe_callback, this, nullptr);
        gst_object_unref(ima_pad);

        // Get the draw vvas_filter element
        draw_probe_ = gst_bin_get_by_name(GST_BIN(pipeline_), "draw");
        if (!draw_probe_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get draw element");
            gst_object_unref(pipeline_);
            rclcpp::shutdown();
            return;
        }

        // Attach a draw probe to the src pad of the draw element
        GstPad *draw_pad = gst_element_get_static_pad(draw_probe_, "sink");
        if (!draw_pad) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get sink pad from draw probe");
            gst_object_unref(draw_probe_);
            gst_object_unref(pipeline_);
            rclcpp::shutdown();
            return;
        }
	
	// Attach callback to the probe
        gst_pad_add_probe(draw_pad, GST_PAD_PROBE_TYPE_BUFFER, draw_probe_callback, this, nullptr);
        gst_object_unref(draw_pad);

	// initialize camera orientation structs
	camera_orientation_.azimuth = 120.12;
	camera_orientation_.elevation = -2.56;

        // Start playing
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    ~GStreamerPipeline() {
        // Free resources
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(draw_probe_);
        gst_object_unref(ima_probe_);
        gst_object_unref(pipeline_);
    }

    void publish_max_bounding_box(const BoundingBox *res_bbox, const BoundingBox *face_bbox) {
        static unsigned int frame_count = 0;

        // Handle only each N-th frame to reduce latency 
        if ((frame_count++) % 2)
            return;

        auto msg = eagle_eye_interfaces::msg::FaceDetect();

        if (face_bbox != nullptr) {
            msg.frame_width = res_bbox->width;
            msg.frame_height = res_bbox->height;
            msg.bbox_x = face_bbox->x;
            msg.bbox_y = face_bbox->y;
            msg.bbox_width = face_bbox->width;
            msg.bbox_height = face_bbox->height;
            msg.face_detected = true;
        } else {
            msg.frame_width = 0;
            msg.frame_height = 0;
            msg.bbox_x = 0;
            msg.bbox_y = 0;
            msg.bbox_width = 0;
            msg.bbox_height = 0;
            msg.face_detected = false;
        }

        face_detect_publisher_->publish(msg);
    }

    static void handle_inference_meta(GstMeta *meta, GStreamerPipeline *gsnode) {
        GstInferenceMeta *inference_meta = reinterpret_cast<GstInferenceMeta *>(meta);

        if (inference_meta && inference_meta->prediction) {
            GstInferencePrediction *prediction = inference_meta->prediction;

            // Parent bbox holds the resolution of the inference frame
            BoundingBox *parent_bbox = &prediction->bbox;

            // Node that holds all detected faces (child nodes)
            GNode *predictions = prediction->predictions;

            if (predictions) {
                GstInferencePrediction *largest_child_pred = nullptr;
                guint max_width = 0;

                for (GNode *node = predictions->children; node; node = node->next) {
                    GstInferencePrediction *child_pred = (GstInferencePrediction *) node->data;

                    if (child_pred->bbox.width > max_width) {
                        max_width = child_pred->bbox.width;
                        largest_child_pred = child_pred;
                    }
                }

                if (largest_child_pred) {
                    gsnode->publish_max_bounding_box(parent_bbox, &largest_child_pred->bbox);
                    return;
                }
            }
        }

        gsnode->publish_max_bounding_box(nullptr, nullptr);
    }

    static GstPadProbeReturn ima_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
        if (!pad) {
            return GST_PAD_PROBE_OK;
        }

        GStreamerPipeline *node = reinterpret_cast<GStreamerPipeline *>(user_data);
        GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
        if (!buffer) {
            node->publish_max_bounding_box(nullptr, nullptr);
            return GST_PAD_PROBE_OK;
        }

        static GType api_type = g_type_from_name("GstVvasInferenceMetaAPI");
        GstMeta *meta = gst_buffer_get_meta(buffer, api_type);

        if (meta) {
            handle_inference_meta(meta, node);
        } else {
            node->publish_max_bounding_box(nullptr, nullptr);
        }

        return GST_PAD_PROBE_OK;
    }

    void inclinometer_callback(const eagle_eye_interfaces::msg::CameraOrientation::SharedPtr msg) {
	{   // received data from subscriber, save them with a lock guard
            std::lock_guard<std::mutex> lock(this->mutex_);

            this->camera_orientation_.azimuth = msg->azimuth;
            this->camera_orientation_.elevation = msg->elevation;
	}
    }

    // sent camera orientation data to gstreamer DRAW element
    static void send_camera_orientation(GstMeta *meta, GStreamerPipeline *gsnode) {
	 
	// struct that be shares between ros2 node and vvas
        static CameraOrientation *cam_orient = nullptr;

        if (!cam_orient) {
            cam_orient = new CameraOrientation();
	}
		 
	// bet inferece meta data struct
        GstInferenceMeta *inference_meta = reinterpret_cast<GstInferenceMeta *>(meta);
			
	if (inference_meta && inference_meta->prediction) {

		// get inference prediction parent data struct
		GstInferencePrediction *prediction = inference_meta->prediction;

		if (prediction && prediction->predictions) {
			
			// get Node that holds all detected faces (child nodes)
			GNode *predictions = prediction->predictions;

			// walk through linked list of all detected faces, we populate all of them just in case..
			for (GNode *node = predictions->children; node; node = node->next) {
				GstInferencePrediction *child_pred = (GstInferencePrediction *) node->data;

				// Attach the shared data struct to the unused pointer of GstInferencePrediction struct
				child_pred->reserved_1 = cam_orient;

				{   // update shared structure with inclinometer data with guard lock
				    std::lock_guard<std::mutex> lock(gsnode->mutex_);

				    cam_orient->azimuth = gsnode->camera_orientation_.azimuth;
				    cam_orient->elevation = gsnode->camera_orientation_.elevation;
				}
			}
		}
	}
    }

    static GstPadProbeReturn draw_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
        if (!pad) {
            return GST_PAD_PROBE_OK;
        }

        GStreamerPipeline *node = reinterpret_cast<GStreamerPipeline *>(user_data);

        GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
        if (!buffer) {
            return GST_PAD_PROBE_OK;
        }

	// get metadata of current buffer
        static GType api_type = g_type_from_name("GstVvasInferenceMetaAPI");
        GstMeta *meta = gst_buffer_get_meta(buffer, api_type);

        if (meta) {
	    send_camera_orientation(meta, node);
        }

        return GST_PAD_PROBE_OK;
    }

private:
    std::mutex mutex_;

    GstElement *pipeline_;
    GstElement *ima_probe_;
    GstElement *draw_probe_;

    CameraOrientation camera_orientation_;

    rclcpp::Publisher<eagle_eye_interfaces::msg::FaceDetect>::SharedPtr face_detect_publisher_;
    rclcpp::Subscription<eagle_eye_interfaces::msg::CameraOrientation>::SharedPtr inclinometer_subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GStreamerPipeline>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
