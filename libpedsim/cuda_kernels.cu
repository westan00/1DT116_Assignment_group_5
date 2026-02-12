#include <cuda_runtime.h>
#include <math.h>

__global__ void cuda_tick_kernel(float *agentX, float *agentY, float *destX,
                                 float *destY, float *desiredX, float *desiredY,
                                 int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    float ax = agentX[i];
    float ay = agentY[i];
    float dx = destX[i];
    float dy = destY[i];

    float diffX = dx - ax;
    float diffY = dy - ay;
    float lenSq = diffX * diffX + diffY * diffY;

    if (lenSq > 0.000001f) {
      float invLen = rsqrtf(lenSq);
      float stepX = diffX * invLen;
      float stepY = diffY * invLen;

      float desX = roundf(ax + stepX);
      float desY = roundf(ay + stepY);

      desiredX[i] = desX;
      desiredY[i] = desY;

      agentX[i] = desX;
      agentY[i] = desY;
    } else {
      desiredX[i] = ax;
      desiredY[i] = ay;
    }
  }
}

extern "C" void launch_cuda_tick(float *agentX, float *agentY, float *destX,
                                 float *destY, float *desiredX, float *desiredY,
                                 int n) {
  int threadsPerBlock = 512;
  int blocksPerGrid = (n + threadsPerBlock - 1) / threadsPerBlock;

  if (n > 0) {
    cuda_tick_kernel<<<blocksPerGrid, threadsPerBlock>>>(
        agentX, agentY, destX, destY, desiredX, desiredY, n);
  }
}
