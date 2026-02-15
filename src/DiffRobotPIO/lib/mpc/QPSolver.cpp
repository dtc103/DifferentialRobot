#include "QPSolver.h"
#include <math.h>

int QPSolver::solve(const float* H, const float* f,
                    const float* lb, const float* ub,
                    float* x, int n,
                    int max_iter, float tol) {
    if(n > MAX_QP_DIM || n <= 0){
        return -1;
    }

    // Stack-allocated working arrays
    float g[MAX_QP_DIM]; // gradient
    float g_prev[MAX_QP_DIM]; // previous gradient (for Barzilai-Borwein step)
    float x_prev[MAX_QP_DIM]; // previous x (for Barzilai-Borwein step)

    // Project initial guess into feasible region
    project(x, lb, ub, n);

    // Compute initial gradient
    compute_gradient(H, f, x, g, n);

    // Initial step size
    float step = 0.01f;

    // Estimate a reasonable initial step size from the Hessian diagonal
    float max_diag = 0.0f;
    for(int i = 0; i < n; i++){
        float d = fabsf(H[i * n + i]);
        if(d > max_diag) max_diag = d;
    }
    if(max_diag > 1e-6f){
        step = 1.0f / max_diag;
    }

    int iter;
    for(iter = 0; iter < max_iter; iter++){

        // Check convergence
        float grad_norm_sq = 0.0f;
        for(int i = 0; i < n; i++){
            float gi = g[i];
            // If x is at a bound and the gradient pushes it further out, the component is zero
            if(x[i] <= lb[i] && gi > 0.0f) gi = 0.0f;
            if(x[i] >= ub[i] && gi < 0.0f) gi = 0.0f;
            grad_norm_sq += gi * gi;
        }

        if(grad_norm_sq < tol * tol){
            break;  // converged
        }

        for(int i = 0; i < n; i++){
            x_prev[i] = x[i];
            g_prev[i] = g[i];
        }

        // Gradient descent step
        for(int i = 0; i < n; i++){
            x[i] -= step * g[i];
        }

        // Project back onto feasible region
        project(x, lb, ub, n);

        // Compute new gradient
        compute_gradient(H, f, x, g, n);

        // Barzilai-Borwein step size update
        float dx_dot_dx = 0.0f;
        float dx_dot_dg = 0.0f;
        for(int i = 0; i < n; i++){
            float dx = x[i] - x_prev[i];
            float dg = g[i] - g_prev[i];
            dx_dot_dx += dx * dx;
            dx_dot_dg += dx * dg;
        }

        if(fabsf(dx_dot_dg) > 1e-12f){
            float new_step = dx_dot_dx / dx_dot_dg;
            // Safeguard: keep step size in a reasonable range
            if(new_step > 1e-8f && new_step < 1e4f){
                step = new_step;
            }
        }
    }

    return iter;
}

void QPSolver::compute_gradient(const float* H, const float* f, const float* x,
                                float* g, int n) {
    // g = H * x + f
    for(int i = 0; i < n; i++){
        float sum = f[i];
        for(int j = 0; j < n; j++){
            sum += H[i * n + j] * x[j];
        }
        g[i] = sum;
    }
}

void QPSolver::project(float* x, const float* lb, const float* ub, int n) {
    for(int i = 0; i < n; i++){
        if(x[i] < lb[i]) x[i] = lb[i];
        if(x[i] > ub[i]) x[i] = ub[i];
    }
}

float QPSolver::norm_sq(const float* v, int n) {
    float sum = 0.0f;
    for(int i = 0; i < n; i++){
        sum += v[i] * v[i];
    }
    return sum;
}
