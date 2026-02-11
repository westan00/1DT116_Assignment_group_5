#include <cuda_runtime.h>
#include <math.h>

__global__ void cuda_tick_kernel(float* agentX, float* agentY, float* destX, float* destY, float* desiredX, float* desiredY, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) {
        float diffX = destX[i] - agentX[i];
        float diffY = destY[i] - agentY[i];
        float len = sqrtf(diffX * diffX + diffY * diffY);
        
        // Only update if there's a significant distance to avoid division by zero or very small numbers
        if (len > 0.001f) { 
            // Calculate step towards destination
            float stepX = diffX / len;
            float stepY = diffY / len;
            
            // Compute desired position by moving one step from current position towards destination
            float desX = agentX[i] + stepX;
            float desY = agentY[i] + stepY;
            
            // Round to nearest integer to match pedestrian simulation grid
            desiredX[i] = roundf(desX);
            desiredY[i] = roundf(desY);
            
            // Agent moves to the desired position
            agentX[i] = desiredX[i];
            agentY[i] = desiredY[i];
        } else {
            // If at destination or very close, stay put (or handle as needed by the simulation logic)
            // For now, keep desired as current position
            desiredX[i] = agentX[i];
            desiredY[i] = agentY[i];
        }
    }
}

extern "C" void launch_cuda_tick(float* agentX, float* agentY, float* destX, float* destY, float* desiredX, float* desiredY, int n) {
    // Define the number of threads per block and blocks per grid
    // A common choice for threads per block is 256 or 512
    int threadsPerBlock = 256; 
    int blocksPerGrid = (n + threadsPerBlock - 1) / threadsPerBlock;

    // Launch the CUDA kernel if there are agents to process
    if (n > 0) {
        cuda_tick_kernel<<<blocksPerGrid, threadsPerBlock>>>(agentX, agentY, destX, destY, desiredX, desiredY, n);
    }
}
