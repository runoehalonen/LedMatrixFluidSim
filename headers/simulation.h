#pragma once
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct Particle
{
    float x, y;
    float velocityX, velocityY;
    float accelerationX, accelerationY;
    int nextInSlot;
    float density;
    float viscosity;
} Particle;

typedef struct SlotCoordinates
{
    int x;
    int y;
} SlotCoordinates;

typedef struct Simulation
{
    float gravity;
    int particleAmount;
    float dampingFactor;
    float maxForceRadius;
    float maxVelocity;

    int boxWidth;
    int boxHeight;

    int matrixWidth;
    int matrixHeight;

    // particles of the simulation. not in any particular order
    Particle *particlePool;

    // active
    int *slotFirstParticleIndexMatrix;

    int particlePoolSize;
} Simulation;

static inline SlotCoordinates checkSlot(float cellSize, float x, float y, int matrixWidth, int matrixHeight);
static inline void applyGravityAndVelocity(Simulation *simulation, float dT, float dx, float dy);
static inline void handleCollisions(Simulation *simulation);
static inline void calculateDensityForces(Simulation *simulation);
static inline void resetSlots(Simulation *simulation);
static inline float pressureFromDensity(float density, Simulation *simulation);
static inline float calculateFluidDensity(float radius, float distancesq);

static inline void simulationInit(Simulation *simulation, int boxWidth, int boxHeight, int particleAmount)
{
    simulation->gravity = 80000.0f;
    simulation->particleAmount = particleAmount;
    // wall bounce velocity damping
    simulation->dampingFactor = 0.5f;
    // max range particles can affect each other
    simulation->maxForceRadius = 60.0f;
    // used to prevent simulation going crazy
    simulation->maxVelocity = 1300.0f;

    simulation->boxWidth = boxWidth;
    simulation->boxHeight = boxHeight;

    // used to split the simulation into a grid to reduce the amount of calculations for each particle
    simulation->matrixWidth = boxWidth / simulation->maxForceRadius;
    simulation->matrixHeight = boxHeight / simulation->maxForceRadius;

    simulation->particlePool = calloc(simulation->particleAmount, sizeof(Particle));
    simulation->slotFirstParticleIndexMatrix = calloc(simulation->matrixWidth * simulation->matrixHeight, sizeof(int));

    // used for loops
    simulation->particlePoolSize = 0;

    memset(simulation->slotFirstParticleIndexMatrix, -1, simulation->matrixWidth * simulation->matrixHeight * sizeof(int));

    // used to build the layer ordering in the particle spawning
    const int layerConstant = 5;
    const int layerHeight = 60;
    const int layerAmount = ceil((float)simulation->particleAmount / layerConstant);
    const int beginY = simulation->boxHeight - 20;
    const int beginX = 15;
    const float distanceUnit = simulation->maxForceRadius * 1.1f;

    int particlesLeft = simulation->particleAmount;
    int createAmount = layerConstant;
    // Initialize particles in layers
    for (int h = 0; h < layerAmount; h++)
    {   
        // staggering layers
        int offsetX = h % 2 == 0 ? 0 : distanceUnit / 2;
        particlesLeft = simulation->particleAmount - (h * layerConstant);
        if (particlesLeft < layerConstant)
        {
            createAmount = particlesLeft;
        }
        for (int x = 0; x < createAmount; x++)
        {
            float particleX = beginX + x * distanceUnit + offsetX;
            float particleY = beginY - h * layerHeight;

            // initializes a new particle
            int newIndex = simulation->particlePoolSize;
            simulation->particlePool[newIndex].x = particleX;
            simulation->particlePool[newIndex].y = particleY;
            simulation->particlePool[newIndex].velocityX = 0.0f;
            simulation->particlePool[newIndex].velocityY = 0.0f;
            simulation->particlePool[newIndex].accelerationX = 0.0f;
            simulation->particlePool[newIndex].accelerationY = 0.0f;
            simulation->particlePool[newIndex].density = 0.0f;
            simulation->particlePool[newIndex].viscosity = 1.0f;


            SlotCoordinates coords = checkSlot(simulation->maxForceRadius, particleX, particleY, simulation->matrixWidth, simulation->matrixHeight);
            // row major index ordering is used to access slotfirstparticleindexmatrix.
            int rowMajorIndex = coords.y * simulation->matrixWidth + coords.x;
            
            // the particles of a single slot are linked together via the variable nextinslot in the struct particle.
            //  if last particle: nextinslot = -1
            simulation->particlePool[newIndex].nextInSlot = simulation->slotFirstParticleIndexMatrix[rowMajorIndex];
            // the row major index location of the current particle is used to set this particle as the first in the slot
            simulation->slotFirstParticleIndexMatrix[rowMajorIndex] = newIndex;
            simulation->particlePoolSize++;
        }
    }
}

static inline void updateSimulation(Simulation *simulation, float deltaTime, float dx, float dy)
{
    // one update cycle
    
    float dT = deltaTime;

    // in a comment because you can induce fun behaviour if you move the window slightly 
    // (deltatime increased due to update not running and movement amplified)
    // otherwise maybe sensible to have a certain max threshold
    //if (dT > 0.016f) dT = 0.016f;

    resetSlots(simulation);
    calculateDensityForces(simulation);
    applyGravityAndVelocity(simulation, dT, dx, dy);
    handleCollisions(simulation);
}

static inline void resetSlots(Simulation *simulation)
{
    // need to clear memory and set it again to have correct slots for each particle
    memset(simulation->slotFirstParticleIndexMatrix, -1, simulation->matrixWidth * simulation->matrixHeight * sizeof(int));
    for (int i = 0; i < simulation->particlePoolSize; i++)
    {
        Particle *particle = &simulation->particlePool[i];
        SlotCoordinates slot = checkSlot(simulation->maxForceRadius, particle->x, particle->y, simulation->matrixWidth, simulation->matrixHeight);
        int rowMajorIndex = slot.y * simulation->matrixWidth + slot.x;
        particle->nextInSlot = simulation->slotFirstParticleIndexMatrix[rowMajorIndex];
        simulation->slotFirstParticleIndexMatrix[rowMajorIndex] = i;
    }
}

static inline void applyGravityAndVelocity(Simulation *simulation, float dT, float dx, float dy)
{
    // used to make the simulation settle a bit faster
    //float dragCoefficient = 1.0f - 1.3f * dT;
    //if (dragCoefficient < 0.0f)
    //    dragCoefficient = 0.0f;

    for (int i = 0; i < simulation->particlePoolSize; i++)
    {
        // the changing direction of gravity should be implemented later.
        simulation->particlePool[i].velocityY += simulation->gravity * dT * dy;
        simulation->particlePool[i].velocityX += simulation->gravity * dT * dx;

        float ax = simulation->particlePool[i].accelerationX;
        float ay = simulation->particlePool[i].accelerationY;

        // velocity incremented by deltatime
        simulation->particlePool[i].velocityX += ax * dT;
        simulation->particlePool[i].velocityY += ay * dT;

        //simulation->particlePool[i].velocityX *= dragCoefficient;
        //simulation->particlePool[i].velocityY *= dragCoefficient;

        float vx = simulation->particlePool[i].velocityX;
        float vy = simulation->particlePool[i].velocityY;

        // capping max velocity with this
        float vLen = sqrtf(vx * vx + vy * vy);
        if (vLen > simulation->maxVelocity)
        {
            float scale = simulation->maxVelocity / vLen;
            simulation->particlePool[i].velocityX = vx * scale;
            simulation->particlePool[i].velocityY = vy * scale;
        }

        // resetting acceleration
        simulation->particlePool[i].accelerationX = 0.0f;
        simulation->particlePool[i].accelerationY = 0.0f;

        simulation->particlePool[i].x += simulation->particlePool[i].velocityX * dT;
        simulation->particlePool[i].y += simulation->particlePool[i].velocityY * dT;
    }
}

static inline void calculateDensityForces(Simulation *simulation)
{
    // first loop that goes over every particle. loops over own slot and neighbouring slots in the simulation
    // the first loop calculates the density for each particle. this density is used in the second loop
    for (int i = 0; i < simulation->particlePoolSize; i++)
    {
        simulation->particlePool[i].density = 0.0f;
        Particle *particle = &simulation->particlePool[i];
        SlotCoordinates slot = checkSlot(simulation->maxForceRadius, particle->x, particle->y, simulation->matrixWidth, simulation->matrixHeight);
        for (int ny = slot.y - 1; ny <= slot.y + 1; ny++)
        {
            for (int nx = slot.x - 1; nx <= slot.x + 1; nx++)
            {
                // culling invalid slots
                if (nx < 0 || nx >= simulation->matrixWidth || ny < 0 || ny >= simulation->matrixHeight)
                    continue;
                int neighborRowMajorIndex = ny * simulation->matrixWidth + nx;
                // culling empty slots
                if (simulation->slotFirstParticleIndexMatrix[neighborRowMajorIndex] == -1)
                    continue;
                int currentParticleIndex = simulation->slotFirstParticleIndexMatrix[neighborRowMajorIndex];
                while (currentParticleIndex != -1)
                {
                    if (currentParticleIndex != i)
                    {
                        // calculates distance and uses that with the max radius to calculate density
                        Particle *neighborParticle = &simulation->particlePool[currentParticleIndex];
                        float dx = neighborParticle->x - particle->x;
                        float dy = neighborParticle->y - particle->y;
                        float r = simulation->maxForceRadius;
                        float distsq = dx * dx + dy * dy;
                        if (distsq > 0.0f && distsq < r*r)
                            particle->density += calculateFluidDensity(r, distsq);
                    }
                    currentParticleIndex = simulation->particlePool[currentParticleIndex].nextInSlot;
                }
            }
        }
    }

    // second loop is mostly similar as it loops through every particle again with nearby slots.
    // calculates the acceleration for each particle based on the first loop
    for (int i = 0; i < simulation->particlePoolSize; i++)
    {
        Particle *particle = &simulation->particlePool[i];
        SlotCoordinates slot = checkSlot(simulation->maxForceRadius, particle->x, particle->y, simulation->matrixWidth, simulation->matrixHeight);
        for (int ny = slot.y - 1; ny <= slot.y + 1; ny++)
        {
            for (int nx = slot.x - 1; nx <= slot.x + 1; nx++)
            {
                if (nx < 0 || nx >= simulation->matrixWidth || ny < 0 || ny >= simulation->matrixHeight)
                    continue;
                int neighborRowMajorIndex = ny * simulation->matrixWidth + nx;
                if (simulation->slotFirstParticleIndexMatrix[neighborRowMajorIndex] == -1)
                    continue;
                int currentParticleIndex = simulation->slotFirstParticleIndexMatrix[neighborRowMajorIndex];
                while (currentParticleIndex != -1)
                {
                    if (currentParticleIndex != i)
                    {
                        Particle *neighborParticle = &simulation->particlePool[currentParticleIndex];
                        float dx = neighborParticle->x - particle->x;
                        float dy = neighborParticle->y - particle->y;
                        float r = simulation->maxForceRadius;
                        float dist = sqrtf(dx * dx + dy * dy);
                        if (dist > 0.0f && dist < r)
                        {
                            // basically vibe coded math. the first one is the correct slope, but 
                            // the uncommented one produces better results
                            float strength = 4000.0f;
                            float slope = strength * dist * (r*r - dist*dist) * (r*r - dist*dist) / (r*r*r*r*r*r);
                            //float slope = strength * dist * (r*r - dist*dist) * (r*r - dist*dist) / (r*r*r*r*r*r) + fmax(0, (-dist + 1));

                            // equalized for both particles.
                            //float sharedPressure = fabsf((pressureFromDensity(particle->density) + pressureFromDensity(neighborParticle->density)) * 0.5f);
                            float sharedPressure = (pressureFromDensity(particle->density, simulation) + pressureFromDensity(neighborParticle->density, simulation)) * 0.5f;

                            // acceleration from pressure and change of density and normalized vector of dx and dy
                            particle->accelerationX += sharedPressure * slope * (dx / dist);
                            particle->accelerationY += sharedPressure * slope * (dy / dist);

                            // viscosity calculations. particles want to move in the same direction more as viscosity is increased
                            float dvx = neighborParticle->velocityX - particle->velocityX;
                            float dvy = neighborParticle->velocityY - particle->velocityY;
                            particle->accelerationX += particle->viscosity * dvx;
                            particle->accelerationY += particle->viscosity * dvy;
                        }
                    }
                    currentParticleIndex = simulation->particlePool[currentParticleIndex].nextInSlot;
                }
            }
        }
    }
}

static inline float pressureFromDensity(float density, Simulation *simulation)
{
    float restDensity = 5.0f * (simulation->maxForceRadius / 30.0f) * (simulation->maxForceRadius / 30.0f);

    // affects compressibility of fluid
    float stiffness = 5000.0f;
    
    return stiffness * (density - restDensity);
}

static inline float calculateFluidDensity(float radius, float distancesq)
{
    float force = fmax(0, radius * radius - distancesq);
    force = force * force * force;
    float normalization = (radius * radius) * (radius * radius) * (radius * radius);
    return force / normalization;
}

static inline void handleCollisions(Simulation *simulation)
{
    // handle collisions between particles and with the boundaries of the box
    for (int i = 0; i < simulation->particlePoolSize; i++)
    {
        if (simulation->particlePool[i].x < 0)
        {
            simulation->particlePool[i].x = 0;
            simulation->particlePool[i].velocityX *= -1 * simulation->dampingFactor;
        }
        if (simulation->particlePool[i].x > simulation->boxWidth)
        {
            simulation->particlePool[i].x = simulation->boxWidth;
            simulation->particlePool[i].velocityX *= -1 * simulation->dampingFactor;
        }
        if (simulation->particlePool[i].y < 0)
        {
            simulation->particlePool[i].y = 0;
            simulation->particlePool[i].velocityY *= -1 * simulation->dampingFactor;
        }
        if (simulation->particlePool[i].y > simulation->boxHeight)
        {
            simulation->particlePool[i].y = simulation->boxHeight;
            simulation->particlePool[i].velocityY *= -1 * simulation->dampingFactor;
        }
    }
}

static inline Particle *getParticleCoordinates(Simulation *simulation)
{
    return (Particle *)simulation->particlePool;
}

static inline void simulationDestroy(Simulation *simulation)
{
    // free allocated memory
    free(simulation->particlePool);
    simulation->particlePool = NULL;
    free(simulation->slotFirstParticleIndexMatrix);
    simulation->slotFirstParticleIndexMatrix = NULL;
}

static inline SlotCoordinates checkSlot(float cellSize, float x, float y, int matrixWidth, int matrixHeight)
{
    // checks what slot the particle is in
    SlotCoordinates coords;
    coords.x = (int)(x / cellSize);
    coords.y = (int)(y / cellSize);
    if (coords.x < 0)
        coords.x = 0;
    if (coords.y < 0)
        coords.y = 0;
    if (coords.x >= matrixWidth)
        coords.x = matrixWidth - 1;
    if (coords.y >= matrixHeight)
        coords.y = matrixHeight - 1;
    return coords;
}

static inline int* getSlotFirstParticleIndexMatrix(Simulation *simulation) 
{
    return simulation->slotFirstParticleIndexMatrix;
}
