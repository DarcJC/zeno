#include "helper_cuda.h"
#include "helper_math.h"
#include <cassert>
#include <cstdio>
#include <cmath>
#include <GL/glut.h>

#define N 64

template <class T>
struct xyz {
    T t;
    __host__ __device__ xyz(T &t) : t(t) {
    }

    __host__ __device__ xyz &operator=(float3 const &r) {
        t.x = r.x;
        t.y = r.y;
        t.z = r.z;
        return *this;
    }

    __host__ __device__ xyz &operator=(xyz const &r) {
        t.x = r.t.x;
        t.y = r.t.y;
        t.z = r.t.z;
        return *this;
    }
};

template <class T>
struct volume {
    T *grid;

    void allocate() {
        size_t size = N * N * N;
        checkCudaErrors(cudaMallocManaged(&grid, size * sizeof(T)));
    }

    void free() {
        checkCudaErrors(cudaFree(grid));
    }

    __host__ __device__ T &at(int i, int j, int k) const {
        return grid[i + j * N + k * N * N];
    }

    __host__ __device__ auto &at(int c, int i, int j, int k) const {
        return at(i, j, k)[c];
    }
};

#define GSL(_, start, end) \
    int _ = (start) + blockDim._ * blockIdx._ + threadIdx._; \
    _ < (end); _ += blockDim._ * gridDim._


static inline __constant__ const int directions[][3] = {{0,0,0},{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},{1,1,1},{-1,-1,-1},{1,1,-1},{-1,-1,1},{1,-1,1},{-1,1,-1},{-1,1,1},{1,-1,-1}};
static inline __constant__ const float weights[] = {2.f/9.f, 1.f/9.f, 1.f/9.f, 1.f/9.f, 1.f/9.f, 1.f/9.f, 1.f/9.f,1.f/72.f, 1.f/72.f, 1.f/72.f, 1.f/72.f, 1.f/72.f, 1.f/72.f, 1.f/72.f, 1.f/72.f};

static_assert(sizeof(weights) / sizeof(weights[0]) == 15);

static inline const float niu = 0.005f;
static inline const float tau = 3.f * niu + 0.5f;
static inline const float inv_tau = 1.f / tau;

struct LBM {
    volume<float4> vel;
    volume<float[16]> f_new;
    volume<float[16]> f_old;

    void allocate() {
        vel.allocate();
        f_new.allocate();
        f_old.allocate();
    }

    __device__ float f_eq(int q, int x, int y, int z) {
        float4 v = vel.at(x, y, z);
        float eu = v.x * directions[q][0]
            + v.y * directions[q][1] + v.z * directions[q][2];
        float uv = v.x * v.x + v.y * v.y + v.z * v.z;
        float term = 1.f + 3.f * eu + 4.5f * eu * eu - 1.5f * uv;
        float feq = weights[q] * v.w * term;
        return feq;
    }
};

__global__ void initialize1(LBM lbm) {
    for (GSL(z, 0, N)) for (GSL(y, 0, N)) for (GSL(x, 0, N)) {
        lbm.vel.at(x, y, z) = make_float4(0.f, 0.f, 0.f, 1.f);
    }
}

__global__ void initialize2(LBM lbm) {
    for (GSL(z, 0, N)) for (GSL(y, 0, N)) for (GSL(x, 0, N)) {
        for (int q = 0; q < 15; q++) {
            float f = lbm.f_eq(q, x, y, z);
            lbm.f_new.at(q, x, y, z) = f;
            lbm.f_old.at(q, x, y, z) = f;
        }
    }
}

__global__ void substep1(LBM lbm) {
    //for (GSL(z, 1, N - 1)) for (GSL(y, 1, N - 1)) for (GSL(x, 1, N - 1)) {
    for (GSL(z, 0, N)) for (GSL(y, 0, N)) for (GSL(x, 0, N)) {
        for (int q = 0; q < 15; q++) {
            //int mdx = x - directions[q][0];
            //int mdy = y - directions[q][1];
            //int mdz = z - directions[q][2];
            int mdx = (x - directions[q][0] + N) % N;
            int mdy = (y - directions[q][1] + N) % N;
            int mdz = (z - directions[q][2] + N) % N;
            lbm.f_new.at(q, x, y, z) = lbm.f_old.at(q, mdx, mdy, mdz)
                * (1.f - inv_tau) + lbm.f_eq(q, mdx, mdy, mdz) * inv_tau;
        }
    }
}

__global__ void substep2(LBM lbm) {
    //for (GSL(z, 1, N - 1)) for (GSL(y, 1, N - 1)) for (GSL(x, 1, N - 1)) {
    for (GSL(z, 0, N)) for (GSL(y, 0, N)) for (GSL(x, 0, N)) {
        float m = 0.f;
        float vx = 0.f, vy = 0.f, vz = 0.f;
        for (int q = 0; q < 15; q++) {
            float f = lbm.f_new.at(q, x, y, z);
            lbm.f_old.at(q, x, y, z) = f;
            vx += f * directions[q][0];
            vy += f * directions[q][1];
            vz += f * directions[q][2];
            m += f;
        }
        float mscale = 1.f / fmaxf(m, 1e-6f);
        vx *= mscale; vy *= mscale; vz *= mscale;
        lbm.vel.at(x, y, z) = make_float4(vx, vy, vz, m);
    }
}

__global__ void applybc1(LBM lbm) {
    //for (GSL(z, 1, N - 1)) for (GSL(y, 1, N - 1)) {
    for (GSL(z, 0, N)) for (GSL(y, 0, N)) {
        lbm.vel.at(0, y, z) = lbm.vel.at(1, y, z);
        lbm.vel.at(0, y, z).x = 0.1f;
        lbm.vel.at(0, y, z).y = 0.f;
        lbm.vel.at(0, y, z).z = 0.f;
        for (int q = 0; q < 15; q++) {
            lbm.f_old.at(q, 0, y, z) =
                lbm.f_eq(q, 0, y, z) - lbm.f_eq(q, 1, y, z)
                + lbm.f_old.at(q, 1, y, z);
        }
        lbm.vel.at(N - 1, y, z) = lbm.vel.at(N - 2, y, z);
        for (int q = 0; q < 15; q++) {
            lbm.f_old.at(q, N - 1, y, z) =
                lbm.f_eq(q, N - 1, y, z) - lbm.f_eq(q, N - 2, y, z)
                + lbm.f_old.at(q, N - 2, y, z);
        }
    }
}

__global__ void applybc2(LBM lbm) {
    for (GSL(z, 0, N)) for (GSL(x, 0, N)) {
        lbm.vel.at(x, 0, z) = lbm.vel.at(x, 1, z);
        lbm.vel.at(x, 0, z).x = 0.f;
        lbm.vel.at(x, 0, z).y = 0.f;
        lbm.vel.at(x, 0, z).z = 0.f;
        for (int q = 0; q < 15; q++) {
            lbm.f_old.at(q, x, 0, z) =
                lbm.f_eq(q, x, 0, z) - lbm.f_eq(q, x, 1, z)
                + lbm.f_old.at(q, x, 1, z);
        }
        lbm.vel.at(x, N - 1, z) = lbm.vel.at(x, N - 2, z);
        lbm.vel.at(x, N - 1, z).x = 0.f;
        lbm.vel.at(x, N - 1, z).y = 0.f;
        lbm.vel.at(x, N - 1, z).z = 0.f;
        for (int q = 0; q < 15; q++) {
            lbm.f_old.at(q, x, N - 1, z) =
                lbm.f_eq(q, x, N - 1, z) - lbm.f_eq(q, x, N - 2, z)
                + lbm.f_old.at(q, x, N - 2, z);
        }
    }
}

__global__ void applybc3(LBM lbm) {
    for (GSL(y, 0, N)) for (GSL(x, 0, N)) {
        lbm.vel.at(x, y, 0) = lbm.vel.at(x, y, 1);
        lbm.vel.at(x, y, 0).x = 0.f;
        lbm.vel.at(x, y, 0).y = 0.f;
        lbm.vel.at(x, y, 0).z = 0.f;
        for (int q = 0; q < 15; q++) {
            lbm.f_old.at(q, x, y, 0) =
                lbm.f_eq(q, x, y, 0) - lbm.f_eq(q, x, y, 1)
                + lbm.f_old.at(q, x, y, 1);
        }
        lbm.vel.at(x, y, N - 1) = lbm.vel.at(x, y, N - 2);
        lbm.vel.at(x, y, N - 1).x = 0.f;
        lbm.vel.at(x, y, N - 1).y = 0.f;
        lbm.vel.at(x, y, N - 1).z = 0.f;
        for (int q = 0; q < 15; q++) {
            lbm.f_old.at(q, x, y, N - 1) =
                lbm.f_eq(q, x, y, N - 1) - lbm.f_eq(q, x, y, N - 2)
                + lbm.f_old.at(q, x, y, N - 2);
        }
    }
}

__global__ void applybc4(LBM lbm) {
    //for (GSL(z, 1, N - 1)) for (GSL(y, 1, N - 1)) for (GSL(x, 1, N - 1)) {
    for (GSL(z, N * 3 / 8, N * 5 / 8))
    for (GSL(y, N * 3 / 8, N * 5 / 8))
    for (GSL(x, N * 1 / 8, N * 3 / 8))
    {
        lbm.vel.at(x, y, z).x = 0.f;
        lbm.vel.at(x, y, z).y = 0.f;
        lbm.vel.at(x, y, z).z = 0.f;
    }
}

LBM lbm;
float *pixels;

void initFunc() {
    lbm.allocate();
    checkCudaErrors(cudaMallocManaged(&pixels, N * N * sizeof(float)));
    initialize1<<<dim3(N / 8, N / 8, N / 8), dim3(8, 8, 8)>>>(lbm);
    initialize2<<<dim3(N / 8, N / 8, N / 8), dim3(8, 8, 8)>>>(lbm);
}

void stepFunc() {
    substep1<<<dim3(N / 8, N / 8, N / 8), dim3(8, 8, 8)>>>(lbm);
    substep2<<<dim3(N / 8, N / 8, N / 8), dim3(8, 8, 8)>>>(lbm);
    applybc1<<<dim3(1, N / 16, N / 16), dim3(1, 16, 16)>>>(lbm);
    applybc2<<<dim3(N / 16, 1, N / 16), dim3(16, 1, 16)>>>(lbm);
    applybc3<<<dim3(N / 16, N / 16, 1), dim3(16, 16, 1)>>>(lbm);
    applybc4<<<dim3(N / 32, N / 32, N / 32), dim3(8, 8, 8)>>>(lbm);
}

__global__ void render(float *pixels, LBM lbm) {
    for (GSL(y, 0, N)) for (GSL(x, 0, N)) {
        float4 v = lbm.vel.at(x, y, N / 2);
        //float val = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
        float val = 4.f * sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
        //float val = v.x * 4.f;
        //float val = v.w * 0.3f;
        pixels[y * N + x] = val;
    }
}

void renderFunc() {
    render<<<dim3(N / 16, N / 16, 1), dim3(16, 16, 1)>>>(pixels, lbm);
    checkCudaErrors(cudaDeviceSynchronize());
    /*printf("03:%f\n", pixels[0 * N + 3]);
    printf("30:%f\n", pixels[3 * N + 0]);
    printf("33:%f\n", pixels[3 * N + 3]);*/
}

void displayFunc() {
    glClear(GL_COLOR_BUFFER_BIT);
    glDrawPixels(N, N, GL_RED, GL_FLOAT, pixels);
    glFlush();
}

#define ITV 0
void timerFunc(int unused) {
    stepFunc();
    renderFunc();
    glutPostRedisplay();
    glutTimerFunc(ITV, timerFunc, 0);
}

void keyboardFunc(unsigned char key, int x, int y) {
    if (key == 27)
        exit(0);
}

int main(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(N, N);
    glutCreateWindow("GLUT Window");
    glutDisplayFunc(displayFunc);
    glutKeyboardFunc(keyboardFunc);
    initFunc();
    renderFunc();
    glutTimerFunc(ITV, timerFunc, 0);
    glutMainLoop();
}
