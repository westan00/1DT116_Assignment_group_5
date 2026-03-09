#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math.h>

#define SIZE 1024
#define CELLSIZE 5
#define SCALED_SIZE (SIZE * CELLSIZE)
#define WEIGHTSUM 273

__constant__ int blur_weights[5][5] = {{1, 4, 7, 4, 1},
                                       {4, 16, 26, 16, 4},
                                       {7, 26, 41, 26, 7},
                                       {4, 16, 26, 16, 4},
                                       {1, 4, 7, 4, 1}};

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

    } else {
      desiredX[i] = ax;
      desiredY[i] = ay;
    }
  }
}

__global__ void cuda_tick_kernel_full(float *agentX, float *agentY,
                                      float *desiredX, float *desiredY,
                                      int *currentWpIdx, int *wpSequences,
                                      int *wpSequencesLen, float *wpX,
                                      float *wpY, float *wpR,
                                      int maxWpsPerAgent, int n) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) {
    int seqPos = currentWpIdx[i];
    int wpId = wpSequences[i * maxWpsPerAgent + seqPos];

    float ax = agentX[i];
    float ay = agentY[i];
    float dx = wpX[wpId];
    float dy = wpY[wpId];
    float dr = wpR[wpId];

    float diffX = dx - ax;
    float diffY = dy - ay;
    float lenSq = diffX * diffX + diffY * diffY;

    if (lenSq < dr * dr) {
      seqPos = (seqPos + 1) % wpSequencesLen[i];
      currentWpIdx[i] = seqPos;

      wpId = wpSequences[i * maxWpsPerAgent + seqPos];
      dx = wpX[wpId];
      dy = wpY[wpId];
      diffX = dx - ax;
      diffY = dy - ay;
      lenSq = diffX * diffX + diffY * diffY;
    }

    if (lenSq > 0.000001f) {
      float invLen = rsqrtf(lenSq);
      float stepX = diffX * invLen;
      float stepY = diffY * invLen;

      float desX = roundf(ax + stepX);
      float desY = roundf(ay + stepY);

      desiredX[i] = desX;
      desiredY[i] = desY;

    } else {
      desiredX[i] = ax;
      desiredY[i] = ay;
    }
  }
}

__global__ void fade_heatmap_kernel(int *heatmap) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x < SIZE && y < SIZE) {
    int idx = y * SIZE + x;
    heatmap[idx] = (int)(heatmap[idx] * 0.80f + 0.5f);
  }
}

__global__ void add_agent_heat_kernel(int *heatmap, float *agentX,
                                      float *agentY, int num_agents) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < num_agents) {
    int x = (int)agentX[i];
    int y = (int)agentY[i];
    if (x >= 0 && x < SIZE && y >= 0 && y < SIZE) {
      if (&heatmap[y * SIZE + x]) {
        atomicExch(&heatmap[y * SIZE + x], 255);
      } else {
        atomicAdd(&heatmap[y * SIZE + x], 40);
      }
    }
  }
}

//__global__ void clamp_heatmap_kernel(int *heatmap) {
// int x = blockIdx.x * blockDim.x + threadIdx.x;
// int y = blockIdx.y * blockDim.y + threadIdx.y;
// if (x < SIZE && y < SIZE) {
// int idx = y * SIZE + x;
// if (heatmap[idx] > 255)
// heatmap[idx] = 255;
//}
//}

__global__ void scale_heatmap_kernel(int *heatmap, int *scaled_heatmap) {
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x < SCALED_SIZE && y < SCALED_SIZE) {
    int hx = x / CELLSIZE;
    int hy = y / CELLSIZE;
    scaled_heatmap[y * SCALED_SIZE + x] = heatmap[hy * SIZE + hx];
  }
}

#define B_DIM 16
#define HALO 2
#define S_DIM (B_DIM + 2 * HALO)

__global__ void blur_heatmap_kernel(int *scaled, int *blurred) {
  __shared__ int sh[S_DIM][S_DIM];

  int tx = threadIdx.x;
  int ty = threadIdx.y;
  int x = (int)blockIdx.x * (int)blockDim.x + tx;
  int y = (int)blockIdx.y * (int)blockDim.y + ty;

  // Efficiently load the 20x20 tile into shared memory using all threads in the
  // block
  for (int tid = ty * (int)blockDim.x + tx; tid < S_DIM * S_DIM;
       tid += (int)blockDim.x * (int)blockDim.y) {
    int ly = tid / S_DIM;
    int lx = tid % S_DIM;
    int gx = (int)blockIdx.x * (int)blockDim.x + lx - HALO;
    int gy = (int)blockIdx.y * (int)blockDim.y + ly - HALO;

    if (gx >= 0 && gx < SCALED_SIZE && gy >= 0 && gy < SCALED_SIZE) {
      sh[ly][lx] = scaled[gy * SCALED_SIZE + gx];
    } else {
      sh[ly][lx] = 0;
    }
  }

  __syncthreads();

  // Compute blur using shared memory tile
  if (x >= 2 && x < SCALED_SIZE - 2 && y >= 2 && y < SCALED_SIZE - 2) {
    int sum = 0;
    for (int i = -2; i <= 2; i++) {
      for (int j = -2; j <= 2; j++) {
        sum += blur_weights[i + 2][j + 2] * sh[ty + HALO + i][tx + HALO + j];
      }
    }
    int value = sum / WEIGHTSUM;
    blurred[y * SCALED_SIZE + x] = 0x00FF0000 | (value << 24);
  }
}

extern "C" void launch_cuda_tick(float *agentX, float *agentY, float *desX,
                                 float *desY, float *desiredX, float *desiredY,
                                 int n) {
  int threadsPerBlock = 512;
  int blocksPerGrid = (n + threadsPerBlock - 1) / threadsPerBlock;

  if (n > 0) {
    cuda_tick_kernel<<<blocksPerGrid, threadsPerBlock>>>(
        agentX, agentY, desX, desY, desiredX, desiredY, n);
  }
}
extern "C" void launch_cuda_tick_full(float *agentX, float *agentY,
                                      float *desiredX, float *desiredY,
                                      int *currentWpIdx, int *wpSequences,
                                      int *wpSequencesLen, float *wpX,
                                      float *wpY, float *wpR,
                                      int maxWpsPerAgent, int n) {
  int threadsPerBlock = 512;
  int blocksPerGrid = (n + threadsPerBlock - 1) / threadsPerBlock;

  if (n > 0) {
    cuda_tick_kernel_full<<<blocksPerGrid, threadsPerBlock>>>(
        agentX, agentY, desiredX, desiredY, currentWpIdx, wpSequences,
        wpSequencesLen, wpX, wpY, wpR, maxWpsPerAgent, n);
  }
}

extern "C" void launch_heatmap_update(int *d_heatmap, int *d_scaled_heatmap,
                                      int *d_blurred_heatmap, float *d_agentX,
                                      float *d_agentY, int num_agents,
                                      cudaStream_t stream) {
  dim3 block(16, 16);
  dim3 grid((SIZE + block.x - 1) / block.x, (SIZE + block.y - 1) / block.y);

  fade_heatmap_kernel<<<grid, block, 0, stream>>>(d_heatmap);

  int threads = 256;
  int blocks = (num_agents + threads - 1) / threads;
  if (num_agents > 0) {
    add_agent_heat_kernel<<<blocks, threads, 0, stream>>>(d_heatmap, d_agentX,
                                                          d_agentY, num_agents);
  }

  // clamp_heatmap_kernel<<<grid, block, 0, stream>>>(d_heatmap);

  dim3 scaled_grid((SCALED_SIZE + block.x - 1) / block.x,
                   (SCALED_SIZE + block.y - 1) / block.y);
  scale_heatmap_kernel<<<scaled_grid, block, 0, stream>>>(d_heatmap,
                                                          d_scaled_heatmap);

  blur_heatmap_kernel<<<scaled_grid, block, 0, stream>>>(d_scaled_heatmap,
                                                         d_blurred_heatmap);
}
