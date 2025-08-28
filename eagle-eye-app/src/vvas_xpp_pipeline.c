/*
 * Copyright 2021 Xilinx, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define VVAS_GLIB_UTILS 1
#include <glib.h>

#include <vvas_core/vvas_memory.h>
#include <vvas_core/vvas_video.h>
#include <vvas/vvas_kernel.h>

extern uint64_t vvas_memory_get_paddr(VvasMemory* vvas_mem);

uint32_t xlnx_kernel_init(VVASKernel *handle);
uint32_t xlnx_kernel_start(VVASKernel *handle, int start,
                           VVASFrame *in[MAX_NUM_OBJECT], VVASFrame *out[MAX_NUM_OBJECT]);
uint32_t xlnx_kernel_deinit(VVASKernel *handle);

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#include <vvas_core/vvas_log.h>
static int log_level = LOG_LEVEL_WARNING;

typedef struct _kern_priv
{
    float mean_r;
    float mean_g;
    float mean_b;
    float scale_r;
    float scale_g;
    float scale_b;
    VvasContext *vctx;
    VvasMemory *params;
    int log_level;
} ResizeKernelPriv;


uint32_t xlnx_kernel_deinit(VVASKernel *handle)
{
  ResizeKernelPriv *kp = (ResizeKernelPriv*)handle->kernel_priv;
  if (kp) {
    if (kp->params) vvas_memory_free(kp->params);
    if (kp->vctx)   vvas_context_destroy(kp->vctx);
    free(kp);
  }
  return 0;
}


#define CK(cond, fmt, ...) do { \
  if (!(cond)) { \
    LOG_MESSAGE(LOG_LEVEL_ERROR, kp ? kp->log_level : LOG_LEVEL_ERROR, \
                "CK FAIL: " fmt, ##__VA_ARGS__); \
    return 1; \
  } \
} while (0)

uint32_t xlnx_kernel_init(VVASKernel *handle)
{
    json_t *jconfig = handle->kernel_config;
    json_t *val; /* kernel config from app */
    ResizeKernelPriv *kernel_priv;
    float *pPtr;

    handle->is_multiprocess = 0;
    kernel_priv = (ResizeKernelPriv *)calloc(1, sizeof(ResizeKernelPriv));
    if (!kernel_priv) {
        printf("Error: Unable to allocate resize kernel memory\n");
    }

    /* parse config */
    val = json_object_get(jconfig, "mean_r");
    if (!val || !json_is_number(val))
        kernel_priv->mean_r = 0;
    else {
        kernel_priv->mean_r = json_number_value(val);
    }
    printf("Resize: mean_r=%f\n", kernel_priv->mean_r);

    val = json_object_get(jconfig, "mean_g");
    if (!val || !json_is_number(val))
        kernel_priv->mean_g = 0;
    else {
        kernel_priv->mean_g = json_number_value(val);
    }
    printf("Resize: mean_g=%f\n", kernel_priv->mean_g);

    val = json_object_get(jconfig, "mean_b");
    if (!val || !json_is_number(val))
        kernel_priv->mean_b = 0;
    else {
        kernel_priv->mean_b = json_number_value(val);
    }
    printf("Resize: mean_b=%f\n", kernel_priv->mean_b);

    /* parse config */
    val = json_object_get(jconfig, "scale_r");
    if (!val || !json_is_number(val))
	kernel_priv->scale_r = 1;
    else
	kernel_priv->scale_r = json_number_value(val);
    printf("Resize: scale_r=%f\n", kernel_priv->scale_r);

    val = json_object_get(jconfig, "scale_g");
    if (!val || !json_is_number(val))
	kernel_priv->scale_g = 1;
    else
	kernel_priv->scale_g = json_number_value(val);
    printf("Resize: scale_g=%f\n", kernel_priv->scale_g);

    val = json_object_get(jconfig, "scale_b");
    if (!val || !json_is_number(val))
	kernel_priv->scale_b = 1;
    else
	kernel_priv->scale_b = json_number_value(val);
    printf("Resize: scale_b=%f\n", kernel_priv->scale_b);

    val = json_object_get (jconfig, "debug_level");
    if (!val || !json_is_integer (val))
        kernel_priv->log_level = LOG_LEVEL_WARNING;
    else
        kernel_priv->log_level = json_integer_value (val);


  ResizeKernelPriv *kp = (ResizeKernelPriv *)calloc(1, sizeof(*kp));
  if (!kp) return 1;

  // defaults (your JSON parsing can override these)
  kp->log_level = LOG_LEVEL_INFO;

  VvasReturnType vret = VVAS_RET_SUCCESS;
  kp->vctx = vvas_context_create(/*dev_idx*/0, /*xclbin_loc*/NULL,
                                 (VvasLogLevel)kp->log_level, &vret);
  if (!kp->vctx || vret != VVAS_RET_SUCCESS) {
    LOG_MESSAGE(LOG_LEVEL_ERROR, kp->log_level, "vvas_context_create failed rc=%d", vret);
    free(kp);
    return 1;
  }

  // allocate params buffer for 6 floats
  const size_t psize = 6 * sizeof(float);
  VvasReturnType r = VVAS_RET_SUCCESS;
  kp->params = vvas_memory_alloc(kp->vctx,
                                 VVAS_INTERNAL_MEMORY,     // host-visible, device-accessible
                                 VVAS_ALLOC_FLAG_NONE,     // no special flags
                                 0,                        // default bank
                                 psize,
                                 &r);
  if (!kp->params || r != VVAS_RET_SUCCESS) {
    LOG_MESSAGE(LOG_LEVEL_ERROR, kp->log_level,
                "vvas_memory_alloc failed: r=%d ptr=%p", r, (void*)kp->params);
    free(kp);
    return 1;
  }

  // map & write the 6 floats
  VvasMemoryMapInfo mi = {0};
  r = vvas_memory_map(kp->params, VVAS_DATA_MAP_WRITE, &mi);
  if (r != VVAS_RET_SUCCESS) {
    LOG_MESSAGE(LOG_LEVEL_ERROR, kp->log_level, "vvas_memory_map failed: r=%d", r);
    vvas_memory_free(kp->params);
    free(kp);
    return 1;
  }

  if (mi.size < psize) {
    LOG_MESSAGE(LOG_LEVEL_ERROR, kp->log_level,
                "params buffer too small: have %zu need %zu", mi.size, psize);
    vvas_memory_unmap(kp->params, &mi);
    vvas_memory_free(kp->params);
    free(kp);
    return 1;
  }

  float *p = (float*)mi.data;
  p[0] = kp->mean_r;
  p[1] = kp->mean_g;
  p[2] = kp->mean_b;
  p[3] = kp->scale_r;
  p[4] = kp->scale_g;
  p[5] = kp->scale_b;

  vvas_memory_unmap(kp->params, &mi);

  // (optional) fetch device address of params if available in your headers
  // NOTE: this may be declared in a private header; if you can't include it,
  // skip printing it.
  #ifdef HAVE_VVAS_MEMORY_GET_PADDR
  uint64_t paddr = vvas_memory_get_paddr(kp->params);
  LOG_MESSAGE(LOG_LEVEL_INFO, kp->log_level, "params dev paddr=0x%lx (size=%zu)", paddr, psize);
  #endif

  handle->kernel_priv = kp;
  return 0;
}

uint32_t xlnx_kernel_start(VVASKernel *handle, int start, VVASFrame *input[MAX_NUM_OBJECT], VVASFrame *output[MAX_NUM_OBJECT])
{
  ResizeKernelPriv *kp = (ResizeKernelPriv *)handle->kernel_priv;
  CK(kp, "kernel_priv NULL");
  CK(input && input[0], "input[0] NULL");
  CK(output && output[0], "output[0] NULL");
  CK(input[0]->n_planes >= 2,  "input n_planes=%u < 2",  input[0]->n_planes);
  CK(output[0]->n_planes >= 1, "output n_planes=%u < 1", output[0]->n_planes);

  CK(input[0]->paddr[0],  "input Y paddr is 0");
  CK(input[0]->paddr[1],  "input UV paddr is 0");
  CK(output[0]->paddr[0], "output base paddr is 0");
  CK(input[0]->props.stride  > 0, "input stride=0");
  CK(output[0]->props.stride > 0, "output stride=0");

  // device address for params (use the API you actually have; if not available,
  // ensure your alloc type yields a device address via the framework)
  uint64_t params_paddr = 0;
  #ifdef HAVE_VVAS_MEMORY_GET_PADDR
  params_paddr = vvas_memory_get_paddr(kp->params);
  #endif
  CK(params_paddr != 0, "params paddr is 0");

  LOG_MESSAGE(LOG_LEVEL_INFO, kp->log_level,
    "pp args: inY=0x%lx inUV=0x%lx out=0x%lx params=0x%lx "
    "in[%ux%u s=%u] out[%ux%u s=%u]",
    input[0]->paddr[0], input[0]->paddr[1], output[0]->paddr[0], params_paddr,
    input[0]->props.width,  input[0]->props.height,  input[0]->props.stride,
    output[0]->props.width, output[0]->props.height, output[0]->props.stride);

  int ret = vvas_kernel_start(handle, "ppppuuuuuu",
                              input[0]->paddr[0],     // img_inp_y
                              input[0]->paddr[1],     // img_inp_uv
                              output[0]->paddr[0],    // img_out (base)
                              params_paddr,           // params
                              input[0]->props.width,
                              input[0]->props.height,
                              input[0]->props.stride,
                              output[0]->props.width,
                              output[0]->props.height,
                              output[0]->props.stride);
  if (ret < 0) {
    LOG_MESSAGE(LOG_LEVEL_ERROR, kp->log_level, "vvas_kernel_start failed: %d", ret);
  }
  return ret;
}

uint32_t xlnx_kernel_done(VVASKernel *handle)
{
    return 0;
}
