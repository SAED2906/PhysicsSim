#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_stdinc.h>

#define SCREEN_WIDTH 1920
#define SCREEN_HEIGHT 1080
#define MAX_VERTICES 10000
#define MAX_OBJECTS 50

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

    int mass;
} Particle;



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
Particle particles[MAX_OBJECTS];


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


void render_simulation(float delta_time, SDL_Renderer *renderer) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    float G = 0.01;

    for (int x = 0; x < MAX_OBJECTS; x++) {
        for (int y = 0; y < MAX_OBJECTS; y++) {
            if (x == y) {
                break;
            }

            float dx = particles[y].pos.x - particles[x].pos.x;
            float dy = particles[y].pos.y - particles[x].pos.y;
            float r_squared = dx * dx + dy * dy;
            float r = sqrt(r_squared);

            printf("P1 {x: %f, y: %f}, P2 {x: %f, y: %f} \t Res: %f, \t Angle: %f\n", 
                particles[x].pos.x, particles[x].pos.y, 
                particles[y].pos.x, particles[y].pos.y, 
                sqrt(r_squared), atan2(dy, dx));

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

            DrawCircle(renderer, particles[x].pos.x - sqrtf(particles[x].mass), particles[x].pos.y - sqrtf(particles[x].mass), sqrtf(particles[x].mass));
            // SDL_RenderDrawPointF(renderer, particles[x].pos.x, particles[x].pos.y);

        }

    }
    SDL_RenderPresent(renderer);
    return;
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

    SDL_Window *window = SDL_CreateWindow("OBJ Viewer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
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

    // particles[0] = (Particle) {{200, 200},{0,0},{0,0},{100}};
    // particles[1] = (Particle) {{600, 600},{0,0},{0,0},{1000}};
    // particles[2] = (Particle) {{250, 500},{0,0},{0,0},{5000}};
    // particles[3] = (Particle) {{900, 690},{0,0},{0,0},{2000}};
    // particles[4] = (Particle) {{1100, 200},{0,0},{0,0},{5000}};
    // particles[5] = (Particle) {{100, 600},{0,0},{0,0},{1000}};
    // particles[6] = (Particle) {{200, 200},{0,0},{0,0},{1200}};
    // particles[7] = (Particle) {{620, 100},{0,0},{0,0},{1050}};
    // particles[8] = (Particle) {{250, 400},{0,0},{0,0},{500}};
    // particles[9] = (Particle) {{200, 700},{0,0},{0,0},{3000}};

    for (int x = 0; x < MAX_OBJECTS; x++) {
        particles[x] = (Particle) {{rand() % (SCREEN_WIDTH + 1), rand() % (SCREEN_HEIGHT + 1)},{0,0},{0,0},{rand() % (50+1)}};
    }

    int running = 1;
    int captureMouse = 1;
    Uint32 last_time = SDL_GetTicks();
    SDL_SetRelativeMouseMode(SDL_TRUE);

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
            }
        }

        render_simulation(delta_time, renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
