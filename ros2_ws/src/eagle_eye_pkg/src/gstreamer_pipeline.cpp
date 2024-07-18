#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include "eagle_eye_interfaces/msg/face_detect.hpp"

#include "eagle_eye_pkg/gst_inference_meta.h"

class GStreamerPipeline : public rclcpp::Node {
public:
    GStreamerPipeline()
        : Node("gstreamer_pipeline") {

	// define node input parameters
	std::string camera_url_ = "";
	this->declare_parameter<std::string>("camera_url", "rtsp://192.168.1.11:554/stream1");
	this->get_parameter("camera_url", camera_url_);

        publisher_ = this->create_publisher<eagle_eye_interfaces::msg::FaceDetect>("face_detect", 10);

        // Initialize GStreamer
        gst_init(nullptr, nullptr);

        // Build the pipeline string
	std::string pipeline_str = "rtspsrc location=" + camera_url_ + " ! "
                                   "rtph265depay ! h265parse ! omxh265dec ! videoconvert ! "
                                   "video/x-raw,format=NV12 ! tee name=t ! queue ! "
                                   "vvas_xmultisrc kconfig=\"/opt/xilinx/kr260-eagle-eye/share/vvas/facedetect/preprocess.json\" ! queue ! "
                                   "vvas_xfilter kernels-config=\"/opt/xilinx/kr260-eagle-eye/share/vvas/facedetect/aiinference.json\" ! "
                                   "ima.sink_master vvas_xmetaaffixer name=ima ima.src_master ! queue name=probe max-size-buffers=1 leaky=2 ! fakesink "
                                   "t. ! queue max-size-buffers=1 leaky=2 ! ima.sink_slave_0 ima.src_slave_0 ! queue ! "
                                   "vvas_xfilter name=\"draw\" kernels-config=\"/opt/xilinx/kr260-eagle-eye/share/vvas/facedetect/drawresult.json\" ! queue ! "
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

        // Get the vvas_xmetaaffixer element
        probe_ = gst_bin_get_by_name(GST_BIN(pipeline_), "probe");
        if (!probe_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get probe element");
            gst_object_unref(pipeline_);
            rclcpp::shutdown();
            return;
        }

        // Attach a probe to the src pad of the probe element
        GstPad *src_pad = gst_element_get_static_pad(probe_, "src");
        if (!src_pad) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get src pad from probe");
            gst_object_unref(probe_);
            gst_object_unref(pipeline_);
            rclcpp::shutdown();
            return;
        }
        gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER, probe_callback, this, nullptr);
        gst_object_unref(src_pad);

        // Start playing
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    ~GStreamerPipeline() {
        // Free resources
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(probe_);
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

        publisher_->publish(msg);
    }

    static void handle_inference_meta(GstMeta *meta, GStreamerPipeline *node) {
        GstInferenceMeta *inference_meta = reinterpret_cast<GstInferenceMeta *>(meta);

        if (inference_meta && inference_meta->prediction) {
            GstInferencePrediction *prediction = inference_meta->prediction;

            // Parent bbox holds the resolution of the inference frame
            BoundingBox *parent_bbox = &prediction->bbox;

            // Note that holds all detected faces (child nodes)
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
                    node->publish_max_bounding_box(parent_bbox, &largest_child_pred->bbox);
                    return;
                }
            }
        }

        node->publish_max_bounding_box(nullptr, nullptr);
    }

    static GstPadProbeReturn probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data) {
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

private:
    GstElement *pipeline_;
    GstElement *probe_;
    rclcpp::Publisher<eagle_eye_interfaces::msg::FaceDetect>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GStreamerPipeline>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
