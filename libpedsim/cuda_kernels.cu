#include <cuda_runtime.h>
#include <math.h>

__global__ void cuda_tick_kernel(float *agentX, float *agentY, float *destX,
                                 float *destY, float *desiredX, float *desiredY,
                                 int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    float diffX = destX[i] - agentX[i];
    float diffY = destY[i] - agentY[i];
    float len = sqrtf(diffX * diffX + diffY * diffY);

    if (len > 0.001f) {
      float stepX = diffX / len;
      float stepY = diffY / len;

      float desX = agentX[i] + stepX;
      float desY = agentY[i] + stepY;

      desiredX[i] = roundf(desX);
      desiredY[i] = roundf(desY);

      agentX[i] = desiredX[i];
      agentY[i] = desiredY[i];
    } else {
      desiredX[i] = agentX[i];
      desiredY[i] = agentY[i];
    }
  }
}

extern "C" void launch_cuda_tick(float *agentX, float *agentY, float *destX,
                                 float *destY, float *desiredX, float *desiredY,
                                 int n) {
  int threadsPerBlock = 256;
  int blocksPerGrid = (n + threadsPerBlock - 1) / threadsPerBlock;

  if (n > 0) {
    cuda_tick_kernel<<<blocksPerGrid, threadsPerBlock>>>(
        agentX, agentY, destX, destY, desiredX, desiredY, n);
  }
}
