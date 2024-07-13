#define MAX_SEGOUTFMT_LEN 6

enum seg_type
{
  SEMANTIC = 0,
  MEDICAL,
  SEG3D,
};

typedef struct _Segmentation {
  enum seg_type type;
  uint32_t width;
  uint32_t height;
  char fmt[MAX_SEGOUTFMT_LEN];
  void *data;
  bool (*free) (void *);
  bool (*copy) (const void *, void *);
} Segmentation;

typedef struct _VvasColorMetadata {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t alpha;
} VvasColorMetadata;

typedef struct _BoundingBox
{
  gint x;
  gint y;
  guint width;
  guint height;
  VvasColorMetadata box_color;
} BoundingBox;

/**
* GstInferencePrediction:
* @prediction_id: unique id for this specific prediction
* @enabled: flag indicating whether or not this prediction should be
* used for further inference
* @bbox: the BoundingBox for this specific prediction
* @classifications: a linked list of GstInfereferenceClassification
* associated to this prediction
* @predictions: a n-ary tree of child predictions within this
* specific prediction. It is recommended to access the tree
* directly, but to use this module's API to interact with the
* children.
* @sub_buffer: A buffer created from the main buffer.
* @bbox_scaled: bbox co-ordinates scaled to root node resolution or not
* @segmentation: contains output of segmentation model wrapped in GstBuffer
* @obj_track_label: This is currently unused, kept for future use
* Abstraction that represents a prediction
*/
typedef struct _GstInferencePrediction GstInferencePrediction;
struct _GstInferencePrediction
{
   /*<private>*/
   GstMiniObject base;
   GMutex mutex;
   /*<public>*/
   guint64 prediction_id;
   gboolean enabled;
   BoundingBox bbox;
   GList * classifications;
   GNode * predictions;
   GstBuffer *sub_buffer;
   gboolean bbox_scaled; /* bbox co-ordinates scaled to root node resolution or not */
   Segmentation segmentation;
   gchar *obj_track_label;
   /* for future extension */
   void * reserved_1;
   void * reserved_2;
   void * reserved_3;
   void * reserved_4;
   void * reserved_5;
};

/**
* Implements the placeholder for inference information.
*/
typedef struct _GstInferenceMeta GstInferenceMeta;
struct _GstInferenceMeta
{
  GstMeta meta;
  GstInferencePrediction *prediction;
  gchar *stream_id;
};

