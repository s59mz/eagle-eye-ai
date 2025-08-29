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

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <vvas_core/vvas_memory.h>
#include <vvas_core/vvas_memory_priv.h>
#include <vvas_core/vvas_video.h>
#include <vvas_core/vvas_log.h>
#include <vvas/vvas_kernel.h>

typedef struct _kern_priv
{
    float mean_r;
    float mean_g;
    float mean_b;
    float scale_r;
    float scale_g;
    float scale_b;
    VvasContext *vctx;
    VVASFrame *params;
    int log_level;
} ResizeKernelPriv;

static int log_level = LOG_LEVEL_WARNING;

uint32_t xlnx_kernel_start(VVASKernel *handle, int start, VVASFrame *input[MAX_NUM_OBJECT], VVASFrame *output[MAX_NUM_OBJECT]);
uint32_t xlnx_kernel_done(VVASKernel *handle);
uint32_t xlnx_kernel_init(VVASKernel *handle);
uint32_t xlnx_kernel_deinit(VVASKernel *handle);

uint32_t xlnx_kernel_deinit(VVASKernel *handle)
{
    ResizeKernelPriv *kp = (ResizeKernelPriv*)handle->kernel_priv;
    if (kp) {
        if (kp->params) 
	    vvas_memory_free(kp->params);
        if (kp->vctx)   
	    vvas_context_destroy(kp->vctx);
        free(kp);
    }
    return 0;
}

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

    VvasReturnType vret = VVAS_RET_SUCCESS;

    kernel_priv->vctx = vvas_context_create(
		    0,      /*dev_idx*/
		    NULL,   /*xclbin_loc*/
                    (VvasLogLevel) LOG_LEVEL_WARNING, 
		    &vret);

    if (!kernel_priv->vctx || vret != VVAS_RET_SUCCESS) {
        printf("vvas_context_create failed rc=%d", vret);
        free(kernel_priv);
        return 1;
    }

    // allocate params buffer for 6 floats
    const size_t psize = 6 * sizeof(float);

    kernel_priv->params = vvas_memory_alloc(kernel_priv->vctx,
                                 VVAS_INTERNAL_MEMORY,     // host-visible, device-accessible
                                 VVAS_ALLOC_FLAG_NONE,     // no special flags
                                 0,                        // default bank
                                 psize,
                                 &vret);

    if (!kernel_priv->params || vret != VVAS_RET_SUCCESS) {
        printf("vvas_memory_alloc failed: r=%d ptr=%p", vret, (void*)kernel_priv->params);
        free(kernel_priv);
        return 1;
    }

    // map & write the 6 floats
    VvasMemoryMapInfo mi = {0};
    vret = vvas_memory_map(kernel_priv->params, VVAS_DATA_MAP_WRITE, &mi);

    if (vret != VVAS_RET_SUCCESS) {
        printf("vvas_memory_map failed: r=%d", vret);
        vvas_memory_free(kernel_priv->params);
        free(kernel_priv);
        return 1;
    }

    if (mi.size < psize) {
        printf("params buffer too small: have %zu need %zu", mi.size, psize);
        vvas_memory_unmap(kernel_priv->params, &mi);
        vvas_memory_free(kernel_priv->params);
        free(kernel_priv);
        return 1;
    }

    pPtr = (float*)mi.data;

    pPtr[0] = (float)kernel_priv->mean_r;  
    pPtr[1] = (float)kernel_priv->mean_g;  
    pPtr[2] = (float)kernel_priv->mean_b;  
    pPtr[3] = (float)kernel_priv->scale_r;  
    pPtr[4] = (float)kernel_priv->scale_g;  
    pPtr[5] = (float)kernel_priv->scale_b;  

    vvas_memory_unmap(kernel_priv->params, &mi);

    handle->kernel_priv = (void *)kernel_priv;

    return 0;
}

uint32_t xlnx_kernel_start(VVASKernel *handle, int start, VVASFrame *input[MAX_NUM_OBJECT], VVASFrame *output[MAX_NUM_OBJECT])
{
    ResizeKernelPriv *kernel_priv;
    kernel_priv = (ResizeKernelPriv *)handle->kernel_priv;

    uint64_t params_paddr = 0;
    params_paddr = vvas_memory_get_paddr(kernel_priv->params);

    int ret = vvas_kernel_start (handle, "ppppuuuuuu", 
        (input[0]->paddr[0]),
        (input[0]->paddr[1]),
        (output[0]->paddr[0]),
	params_paddr,
        (input[0]->props.width),
        (input[0]->props.height),
        (input[0]->props.stride),
        (output[0]->props.width),
        (output[0]->props.height),
        (output[0]->props.stride)
        );
    if (ret < 0) {
      printf("Preprocess: failed to issue execute command");
      return ret;
    }

    /* wait for kernel completion */
    ret = vvas_kernel_done (handle, 1000);
    if (ret < 0) {
      printf("Error: Preprocess: failed to receive response from kernel");
      return ret;
    } 

    return ret;
}

uint32_t xlnx_kernel_done(VVASKernel *handle)
{
    return 0;
}
