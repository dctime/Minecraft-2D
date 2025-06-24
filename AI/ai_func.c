#include "digit_cnn.h"
#include "digit_cnn_data.h"
#include <stdio.h>
#include "stm324xg_lcd_sklin.h"



static ai_handle g_network = AI_HANDLE_NULL;
static ai_buffer* g_inputs;
static ai_buffer* g_outputs;

AI_ALIGNED(4) static ai_u8 activations[AI_DIGIT_CNN_DATA_ACTIVATIONS_SIZE];

bool init_cnn() {
    ai_error err;
    ai_network_params params;

    // Step 1: Create network once
    err = ai_digit_cnn_create(&g_network, AI_DIGIT_CNN_DATA_CONFIG);
    if (err.type != AI_ERROR_NONE) return false;

    // Step 2: Bind weights and activations
    if (!ai_digit_cnn_data_params_get(&params)) return false;
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, 0, activations);
    if (!ai_digit_cnn_init(g_network, &params)) return false;

    // Step 3: Store input/output buffers
    g_inputs = ai_digit_cnn_inputs_get(g_network, NULL);
    g_outputs = ai_digit_cnn_outputs_get(g_network, NULL);

    return true;
}

void run_cnn(float* image28x28, float* probCircle, float* probSquare, float* probTriangle) {
    // Copy input
    g_inputs[0].data = image28x28;

    // Run inference
    if (ai_digit_cnn_run(g_network, g_inputs, g_outputs) != 1) {while(1);}

    float* output = (float*)g_outputs[0].data;
    int max_index = 0;
		*probCircle = output[0];
		*probSquare = output[1];
		*probTriangle = output[2];
}

void free_cnn() {
    if (g_network != AI_HANDLE_NULL) {
        ai_digit_cnn_destroy(g_network);
        g_network = AI_HANDLE_NULL;
    }
}