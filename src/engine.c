#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_stdinc.h>
#include <pthread.h>



#define SCREEN_WIDTH 1920//800
#define SCREEN_HEIGHT 1080//600
#define MAX_VERTICES 10000
#define MAX_OBJECTS 15000

// SPH Constants
#define MAX_PARTICLES 2000



#define M_PI 3.14159265358979323846
// #endif


float maxa = 0;
float mina = 0;

int zoom = 1;




typedef struct {
    float x, y, z;
} Vec3;

typedef struct {
    float x, y;
} Vec2;

typedef struct {
    float x, y, z;
    float pitch, yaw;
} Camera;

typedef struct {
    Vec2 pos;
    Vec2 vel;
    Vec2 acc;
    float mass;
    float density;
    float pressure;
} Particle;

Particle particles[MAX_PARTICLES];


void DrawCircle(SDL_Renderer* renderer, int32_t centreX, int32_t centreY, int32_t radius)
{
    const int32_t diameter = (radius * 2);

    int32_t x = (radius - 1);
    int32_t y = 0;
    int32_t tx = 1;
    int32_t ty = 1;
    int32_t error = (tx - diameter);

    while (x >= y)
    {
    SDL_RenderDrawPoint(renderer, centreX + x, centreY - y);
    SDL_RenderDrawPoint(renderer, centreX + x, centreY + y);
    SDL_RenderDrawPoint(renderer, centreX - x, centreY - y);
    SDL_RenderDrawPoint(renderer, centreX - x, centreY + y);
    SDL_RenderDrawPoint(renderer, centreX + y, centreY - x);
    SDL_RenderDrawPoint(renderer, centreX + y, centreY + x);
    SDL_RenderDrawPoint(renderer, centreX - y, centreY - x);
    SDL_RenderDrawPoint(renderer, centreX - y, centreY + x);

      if (error <= 0)
      {
      	++y;
      	error += ty;
      	ty += 2;
      }

      if (error > 0)
      {
      	--x;
      	tx += 2;
      	error += (tx - diameter);
      }

    }
    }



int object_count = 0;
SDL_Vertex verts[MAX_VERTICES * 3];
int vert_count = 0;
// Particle particles[MAX_OBJECTS];


Vec2 project(Vec3 point, float fov, float aspect, float near, float far) {
    Vec2 screen_point;
    if (point.z == 0) point.z = 0.001f;
    float f = 1.0f / tan(fov / 2.0f);
    screen_point.x = (point.x * f) / (aspect * point.z);
    screen_point.y = (point.y * f) / point.z;
    return screen_point;
}

Vec3 subtract(Vec3 a, Vec3 b) {
    Vec3 result = {a.x - b.x, a.y - b.y, a.z - b.z};
    return result;
}

Vec3 crossProduct(Vec3 a, Vec3 b) {
    Vec3 result = {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
    return result;
}

float dotProduct(Vec2 a, Vec2 b) {
    return a.x * b.x + a.y * b.y;
}

void check_collisons(Vec2 *vec) {
    // Check if within bounds
}

float length(Vec2 a) { 
    return sqrtf(a.x * a.x + a.y * a.y);
}

float dist(Vec2 a, Vec2 b) {
    return sqrtf((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

float angle_between(Vec2 a, Vec2 b) {
    return acosf(dotProduct(a, b)/(length(a)*length(b))) + 3.141/4;
}

#define MAX_THREADS 12 // Adjust based on your CPU cores

typedef struct {
    float *acc_x;
    float *acc_y;
} ThreadAccel;

typedef struct {
    int x;
    int y;
} CollisionPair;

ThreadAccel thread_accels[MAX_THREADS];
CollisionPair *collision_pairs;
int collision_count = 0;
pthread_mutex_t collision_mutex;

typedef struct {
    int start_x;
    int end_x;
    float delta_time;
    float G;
    int thread_id;
} ThreadArgs;

void *compute_forces(void *args) {
    ThreadArgs *targs = (ThreadArgs *)args;
    int start_x = targs->start_x;
    int end_x = targs->end_x;
    float delta_time = targs->delta_time;
    float G = targs->G;
    int tid = targs->thread_id;

    for (int x = start_x; x < end_x; x++) {
        if (particles[x].mass <= 0) continue;

        for (int y = x + 1; y < MAX_OBJECTS; y++) {
            if (particles[y].mass <= 0) continue;

            float dx = particles[y].pos.x - particles[x].pos.x;
            float dy = particles[y].pos.y - particles[x].pos.y;
            float r_squared = dx*dx + dy*dy;
            float r = sqrt(r_squared);

            // Collision detection
            if (r < (sqrtf(particles[x].mass) + sqrtf(particles[y].mass))) {
                pthread_mutex_lock(&collision_mutex);
                collision_pairs = realloc(collision_pairs, sizeof(CollisionPair) * (collision_count + 1));
                collision_pairs[collision_count++] = (CollisionPair){x, y};
                pthread_mutex_unlock(&collision_mutex);
                continue;
            }

            if (r_squared > 0) {
                float inv_r_cubed = G / r_squared;
                float ax = particles[y].mass * dx * inv_r_cubed;
                float ay = particles[y].mass * dy * inv_r_cubed;

                // Accumulate accelerations (Newton's third law)
                thread_accels[tid].acc_x[x] += ax;
                thread_accels[tid].acc_y[x] += ay;
                thread_accels[tid].acc_x[y] -= (particles[x].mass / particles[y].mass) * ax;
                thread_accels[tid].acc_y[y] -= (particles[x].mass / particles[y].mass) * ay;
            }
        }
    }
    return NULL;
}

void handle_collisions() {
    for (int i = 0; i < collision_count; i++) {
        int x = collision_pairs[i].x;
        int y = collision_pairs[i].y;

        if (particles[x].mass <= 0 || particles[y].mass <= 0) continue;

        // Determine which particle is larger
        if (sqrtf(particles[x].mass) >= sqrtf(particles[y].mass)) {
            particles[x].mass += particles[y].mass;
            particles[y].mass = 0;
            // Reset particle y
            particles[y] = (Particle){
                .pos = {(rand() % (SCREEN_WIDTH + 1)) * zoom, (rand() % (SCREEN_HEIGHT + 1)) * zoom},
                .vel = {((rand() % 2) - 0.5f), ((rand() % 2) - 0.5f)},
                .acc = {0, 0},
                .mass = 2
            };
        } else {
            particles[y].mass += particles[x].mass;
            particles[x].mass = 0;
            // Reset particle x
            particles[x] = (Particle){
                .pos = {(rand() % (SCREEN_WIDTH + 1)) * zoom, (rand() % (SCREEN_HEIGHT + 1)) * zoom},
                .vel = {((rand() % 2) - 0.5f), ((rand() % 2) - 0.5f)},
                .acc = {0, 0},
                .mass = 2
            };
        }
    }
    collision_count = 0; // Reset for next frame
}

void render_grav_simulation(float delta_time, SDL_Renderer *renderer, int zoom, float x_offset, float y_offset) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    float G = 0.5f;
    pthread_t threads[MAX_THREADS];
    ThreadArgs targs[MAX_THREADS];
    int chunk = MAX_OBJECTS / MAX_THREADS;

    // Initialize thread acceleration buffers
    for (int t = 0; t < MAX_THREADS; t++) {
        thread_accels[t].acc_x = calloc(MAX_OBJECTS, sizeof(float));
        thread_accels[t].acc_y = calloc(MAX_OBJECTS, sizeof(float));
    }

    // Initialize collision data
    collision_pairs = malloc(0);
    pthread_mutex_init(&collision_mutex, NULL);

    // Create threads
    for (int t = 0; t < MAX_THREADS; t++) {
        targs[t] = (ThreadArgs){
            .start_x = t * chunk,
            .end_x = (t == MAX_THREADS - 1) ? MAX_OBJECTS : (t + 1) * chunk,
            .delta_time = delta_time,
            .G = G,
            .thread_id = t
        };
        pthread_create(&threads[t], NULL, compute_forces, &targs[t]);
    }

    // Join threads
    for (int t = 0; t < MAX_THREADS; t++) {
        pthread_join(threads[t], NULL);
    }

    // Combine accelerations from all threads
    for (int i = 0; i < MAX_OBJECTS; i++) {
        particles[i].acc.x = particles[i].acc.y = 0;
        for (int t = 0; t < MAX_THREADS; t++) {
            particles[i].acc.x += thread_accels[t].acc_x[i];
            particles[i].acc.y += thread_accels[t].acc_y[i];
        }
    }

    for (int i = 0; i < MAX_OBJECTS; i++) {
        if (particles[i].mass <= 0) continue;

        particles[i].vel.x += particles[i].acc.x * delta_time;
        particles[i].vel.y += particles[i].acc.y * delta_time;
        particles[i].pos.x += particles[i].vel.x * delta_time;
        particles[i].pos.y += particles[i].vel.y * delta_time;
    }

    handle_collisions();

    
    for (int i = 0; i < MAX_OBJECTS; i++) {
        if (particles[i].mass <= 0) continue;

        float px = particles[i].pos.x / zoom + x_offset;
        float py = particles[i].pos.y / zoom + y_offset;
        float radius = sqrtf(particles[i].mass) / zoom;
        if ((sqrtf(particles[i].acc.x*particles[i].acc.x + particles[i].acc.y*particles[i].acc.y)*30) > 255) {
            SDL_SetRenderDrawColor(renderer, 255, 128, 255, 255);
        } else {
            SDL_SetRenderDrawColor(renderer, (sqrtf(particles[i].acc.x*particles[i].acc.x + particles[i].acc.y*particles[i].acc.y)*30), 128, 255, 255);
        }
        if (radius >= 1) {
            DrawCircle(renderer, px, py, radius);
        } else {
            SDL_RenderDrawPointF(renderer, px, py);
        }
    }

    SDL_RenderPresent(renderer);

    for (int t = 0; t < MAX_THREADS; t++) {
        free(thread_accels[t].acc_x);
        free(thread_accels[t].acc_y);
    }
    free(collision_pairs);
    pthread_mutex_destroy(&collision_mutex);
}

void rrender_grav_simulation(float delta_time, SDL_Renderer *renderer, int zoom, float x_offset, float y_offset) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    delta_time *= 1;

    float G = 0.05;

    float max_mass = 0;

    

    // for (int x = 0; x < SCREEN_WIDTH; x++) {
    //     for (int y = 0; y < SCREEN_HEIGHT; y++) {
    //         float acc = 0;
    //         for (int p = 0; p < MAX_OBJECTS; p++) {
    //             float dx = x - particles[p].pos.x;
    //             float dy = y - particles[p].pos.y;
    //             float r_squared = dx * dx + dy * dy;
    //             float r = sqrt(r_squared);

    //             if (r_squared > 0) {
    //                 float inv_r_cubed = G / (r*r);
                    
    //                 acc += particles[p].mass * dx * inv_r_cubed;
    //                 acc += particles[p].mass * dy * inv_r_cubed;
                    
    //             }// } else {
    //             //     acc = 0;
    //             // }
    //             // printf("x:%d, y:%d, r:%f, acc:%f\n",x,y,r,acc);
    //             // exit(0);
    //         }

    //         if (acc < 0) {
    //             acc *= -1;
    //         }

    //         SDL_SetRenderDrawColor(renderer, acc*100, acc*100, acc*100, 255);
    //         SDL_RenderDrawPointF(renderer, x, y);


    //         // if (acc > maxa) {
    //         //     maxa = acc;
    //         // }

    //         // if (acc < mina) {
    //         //     mina = acc;
    //         // }

            
    //     }
    // }

    // printf("max: %f, min:%f\n", maxa, mina);


    for (int x = 0; x < MAX_OBJECTS; x++) {
        // if (particles[x].mass > max_mass) {
        //     max_mass = particles[x].mass;
        //     x_offset = -particles[x].pos.x/zoom + (SCREEN_WIDTH*zoom)/2;
        //     y_offset = -particles[x].pos.y/zoom + (SCREEN_HEIGHT*zoom)/2;
        // }
        for (int y = x+1; y < MAX_OBJECTS; y++) {
                if (particles[x].mass == 0) {
                    break;
                }
                if (particles[y].mass == 0) {
                    continue;
                }
                // if (x == y) {
                //     break;
                // }
                if (x==0) {
                    particles[x].pos.x = zoom * (SCREEN_WIDTH/2) + x_offset;
                    particles[x].pos.y = zoom * (SCREEN_HEIGHT/2) + y_offset;
                }

                float dx = particles[y].pos.x - particles[x].pos.x;
                float dy = particles[y].pos.y - particles[x].pos.y;
                float r_squared = dx * dx + dy * dy;
                float r = sqrt(r_squared);

                if (r < sqrtf(particles[x].mass) + sqrtf(particles[y].mass)) {
                    if (sqrtf(particles[x].mass) >= sqrtf(particles[y].mass)) {
                        particles[x].mass += particles[y].mass;
                        particles[y].mass = 0;

                        particles[y] = (Particle) {{(rand() % (SCREEN_WIDTH + 1))*zoom, (rand() % (SCREEN_HEIGHT + 1))*zoom},{((rand() % 2)-1)/2,((rand() % 2)-1)/2},{0,0},{2}};
                        continue;
                    } else {
                        particles[y].mass += particles[x].mass;
                        particles[x].mass = 0;

                        particles[x] = (Particle) {{(rand() % (SCREEN_WIDTH + 1))*zoom, (rand() % (SCREEN_HEIGHT + 1))*zoom},{((rand() % (2)-1))/10,((rand() % (2))-1)/10},{0,0},{2}};
                        continue;
                    }
                }

                // printf("P1 {x: %f, y: %f}, P2 {x: %f, y: %f} \t Res: %f, \t Angle: %f\n", 
                //     particles[x].pos.x, particles[x].pos.y, 
                //     particles[y].pos.x, particles[y].pos.y, 
                //     sqrt(r_squared), atan2(dy, dx));

                if (r_squared > 0) {
                    float inv_r_cubed = G / (r*r);
                    
                    particles[x].acc.x = particles[y].mass * dx * inv_r_cubed;
                    particles[x].acc.y = particles[y].mass * dy * inv_r_cubed;
                    
                    particles[y].acc.x = -particles[x].mass * dx * inv_r_cubed;
                    particles[y].acc.y = -particles[x].mass * dy * inv_r_cubed;
                } else {
                    particles[x].acc.x = particles[x].acc.y = 0;
                    particles[y].acc.x = particles[y].acc.y = 0;
                }

                particles[x].vel.x += particles[x].acc.x * delta_time;
                particles[x].vel.y += particles[x].acc.y * delta_time;
                particles[x].pos.x += particles[x].vel.x * delta_time;
                particles[x].pos.y += particles[x].vel.y * delta_time;

                particles[y].vel.x += particles[y].acc.x * delta_time;
                particles[y].vel.y += particles[y].acc.y * delta_time;
                particles[y].pos.x += particles[y].vel.x * delta_time;
                particles[y].pos.y += particles[y].vel.y * delta_time;

                if (x==0) {
                    particles[x].pos.x = zoom * (SCREEN_WIDTH/2);
                    particles[x].pos.y = zoom * (SCREEN_HEIGHT/2);
                    if (sqrtf(particles[x].mass)/zoom >= 1) {
                        DrawCircle(renderer, particles[x].pos.x/zoom + x_offset, particles[x].pos.y/zoom + y_offset, sqrtf(particles[x].mass)/zoom);
                    } else {
                        SDL_RenderDrawPointF(renderer, particles[x].pos.x/zoom + x_offset, particles[x].pos.y/zoom + y_offset);
                    }
                } else {
                    if (sqrtf(particles[x].mass)/zoom >= 1) {
                        DrawCircle(renderer, particles[x].pos.x/zoom + x_offset, particles[x].pos.y/zoom + y_offset, sqrtf(particles[x].mass)/zoom);
                    } else {
                        SDL_RenderDrawPointF(renderer, particles[x].pos.x/zoom + x_offset, particles[x].pos.y/zoom + y_offset);
                    }
                }
                // SDL_RenderDrawPointF(renderer, particles[x].pos.x, particles[x].pos.y);

            }

    }
    SDL_RenderPresent(renderer);
    return;
}


void render_fluid_simulation(float delta_time, SDL_Renderer *renderer) {

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    const float h = 16.0f;
    const float REST_DENSITY = 50.0f;
    const float PRESSURE_K = 2.5f;
    const float VISCOSITY = 0.1f;
    const Vec2 GRAVITY = {0.0f, 98.1f};
    const float BOUND_DAMPING = -0.5f; 

    for (int i = 0; i < MAX_PARTICLES; i++) {
        particles[i].density = 0.0f;

        for (int j = 0; j < MAX_PARTICLES; j++) {
            Vec2 delta = {
                particles[j].pos.x - particles[i].pos.x,
                particles[j].pos.y - particles[i].pos.y
            };
            float dist_sq = delta.x*delta.x + delta.y*delta.y;
            
            if (dist_sq < h*h) {
                float r = sqrtf(dist_sq);
                float h_sq = h*h;
                // Poly6 kernel
                float volume = (315.0f / (64.0f * M_PI * powf(h, 9))) * powf(h_sq - dist_sq, 3);
                particles[i].density += particles[j].mass * volume;
            }
        }
        
        particles[i].pressure = PRESSURE_K * (particles[i].density - REST_DENSITY);
    }

    for (int i = 0; i < MAX_PARTICLES; i++) {
        Vec2 pressure_force = {0};
        Vec2 viscosity_force = {0};
        
        for (int j = 0; j < MAX_PARTICLES; j++) {
            if (i == j) continue;
            
            Vec2 delta = {
                particles[j].pos.x - particles[i].pos.x,
                particles[j].pos.y - particles[i].pos.y
            };
            float dist_sq = delta.x*delta.x + delta.y*delta.y;
            
            if (dist_sq < h*h) {
                float r = sqrtf(dist_sq);
                if (r < 0.0001f) continue;
                
                float gradW = -45.0f / (M_PI * powf(h, 6)) * powf(h - r, 2);
                float pressure_term = (particles[i].pressure + particles[j].pressure) / (2.0f * particles[j].density);
                Vec2 pressure_dir = {delta.x/r, delta.y/r};
                
                pressure_force.x += pressure_term * gradW * particles[j].mass * pressure_dir.x;
                pressure_force.y += pressure_term * gradW * particles[j].mass * pressure_dir.y;
                
                Vec2 vel_diff = {
                    particles[j].vel.x - particles[i].vel.x,
                    particles[j].vel.y - particles[i].vel.y
                };
                float laplacianW = 45.0f / (M_PI * powf(h, 6)) * (h - r);
                viscosity_force.x += VISCOSITY * particles[j].mass * vel_diff.x * laplacianW / particles[j].density;
                viscosity_force.y += VISCOSITY * particles[j].mass * vel_diff.y * laplacianW / particles[j].density;
            }
        }
 
        particles[i].acc.x = (-pressure_force.x + viscosity_force.x) / particles[i].density + GRAVITY.x;
        particles[i].acc.y = (-pressure_force.y + viscosity_force.y) / particles[i].density + GRAVITY.y;
    }

    for (int i = 0; i < MAX_PARTICLES; i++) {

        particles[i].vel.x += particles[i].acc.x * delta_time;
        particles[i].vel.y += particles[i].acc.y * delta_time;
    
        particles[i].pos.x += particles[i].vel.x * delta_time;
        particles[i].pos.y += particles[i].vel.y * delta_time;
        
        if (particles[i].pos.x < 0) {
            particles[i].pos.x = 0;
            particles[i].vel.x *= BOUND_DAMPING;
        }
        if (particles[i].pos.x > SCREEN_WIDTH) {
            particles[i].pos.x = SCREEN_WIDTH;
            particles[i].vel.x *= BOUND_DAMPING;
        }
        if (particles[i].pos.y < 0) {
            particles[i].pos.y = 0;
            particles[i].vel.y *= BOUND_DAMPING;
        }
        if (particles[i].pos.y > SCREEN_HEIGHT) {
            particles[i].pos.y = SCREEN_HEIGHT-5;
            particles[i].vel.y *= BOUND_DAMPING;
        }

        if (particles[i].pos.y > SCREEN_HEIGHT - 30) {
            particles[i].acc.y -= 100.0f;
        }

    }

    SDL_SetRenderDrawColor(renderer, 30, 144, 255, 255);
    for (int i = 0; i < MAX_PARTICLES; i++) {
        SDL_Rect particle_rect = {
            (int)particles[i].pos.x - 2,
            (int)particles[i].pos.y - 2,
            5, 5
        };
        SDL_RenderFillRect(renderer, &particle_rect);
    }
    SDL_RenderPresent(renderer);
}

Vec3 calculateNormal(Vec3 v1, Vec3 v2, Vec3 v3) {
    Vec3 edge1 = subtract(v2, v1);
    Vec3 edge2 = subtract(v3, v1);
    return crossProduct(edge1, edge2);
}

int main(int argc, char *argv[]) {

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        printf("Error: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow("PhysSim", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (window == NULL) {
        printf("Error: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == NULL) {
        SDL_DestroyWindow(window);
        printf("Error: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    Camera camera = {0, 0, -5, 0, 0};
    float fov = 3.14159f / 4.0f;
    float aspect = (float)SCREEN_WIDTH / (float)SCREEN_HEIGHT;
    float near = 0.1f;
    float far = 100.0f;

    // create_particles(
    // (Vec2){100, 100},  // Start position
    // 20,                 // rows (y-axis count)
    // 20,                 // cols (x-axis count)
    // 15.0f,             // spacing (must < KERNEL_RADIUS)
    // 100.0f               // mass
    // );

    particles[0] = (Particle) {{SCREEN_WIDTH/2, SCREEN_HEIGHT/2},{0,0},{0,0},{300}};
    // particles[1] = (Particle) {{SCREEN_WIDTH/2, (SCREEN_HEIGHT*3)/4},{-10,0},{0,0},{40}};
    // particles[2] = (Particle) {{SCREEN_WIDTH/2, (SCREEN_HEIGHT)/4},{10,0},{0,0},{90}};
    // particles[3] = (Particle) {{SCREEN_WIDTH/2, (SCREEN_HEIGHT*3)/4 - 20},{3,0},{0,0},{5}};
    // particles[4] = (Particle) {{1100, 200},{0,0},{0,0},{1}};
    // particles[5] = (Particle) {{100, 600},{0,0},{0,0},{1}};
    // particles[6] = (Particle) {{200, 200},{0,0},{0,0},{1}};
    // particles[7] = (Particle) {{620, 100},{0,0},{0,0},{1050}};
    // particles[8] = (Particle) {{250, 400},{0,0},{0,0},{500}};
    // particles[9] = (Particle) {{200, 700},{0,0},{0,0},{3000}};

    for (int x = 1; x < MAX_PARTICLES; x++) {
        particles[x] = (Particle) {{rand() % (SCREEN_WIDTH + 1), rand() % (SCREEN_HEIGHT + 1)},{((rand() % (2))-1)/10,((rand() % (2))-1)/10},{0,0},{2}};//rand() % (100+1)}
    }

    // printf("PI VALUE: %.15f\n", M_PI);  // Should show 3.14159...

    int running = 1;
    int captureMouse = 1;
    Uint32 last_time = SDL_GetTicks();
    SDL_SetRelativeMouseMode(SDL_TRUE);

    int speedup = 0;
    // int zoom = 1;

    int old_s = speedup;
    int old_z = zoom;

    float x_offset = 0.0f;
    float y_offset = 0.0f;

    while (running) {
        Uint32 current_time = SDL_GetTicks();
        float delta_time = (current_time - last_time) / 1000.0f;
        last_time = current_time;

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = 0;
            } else if (event.type == SDL_MOUSEMOTION) {
                if (captureMouse) {
                    // handle_mouse_movement(&camera, event.motion.xrel, event.motion.yrel);
                }
            } else if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_ESCAPE)) {
                running = 0;
                if (captureMouse == 0) {
                    captureMouse = 1;
                } else {
                    captureMouse = 0;
                }
            } else if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_LEFT)) {
                speedup -= 1;
            } else if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_RIGHT)) {
                speedup += 1;
            } else if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_UP)) {
                zoom += 1;
            } else if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_DOWN)) {
                zoom -= 1;
                if (zoom <= 0) zoom = 1;
            } else if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_0)) {
                particles[0].mass *= 2;
            } else if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_9)) {
                particles[0].mass /= 2;
            } else if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_w)) {
                y_offset += 4*zoom;
            } else if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_s)) {
                y_offset -= 4*zoom;
            } else if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_a)) {
                x_offset += 4*zoom;
            } else if ((event.type == SDL_KEYDOWN) && (event.key.keysym.sym == SDLK_d)) {
                x_offset -= 4*zoom;
            } 

        }

        render_grav_simulation(delta_time * speedup, renderer, zoom, x_offset, y_offset);
        // render_fluid_simulation(delta_time, renderer);

        if (old_z != zoom || old_s != speedup) {
            printf("Delta Time: %f, Speedup: %d, Zoom: %d\n", delta_time, speedup, zoom);
            old_s = speedup;
            old_z = zoom;
        }
        
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
