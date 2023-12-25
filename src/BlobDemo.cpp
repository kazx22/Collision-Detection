/*
 * The Blob demo.
 *
 */
#include <gl/glut.h>
#include "app.h"
#include "coreMath.h"
#include "pcontacts.h"
#include "pworld.h"
#include <stdio.h>
#include <cassert>
#include <iostream>


using namespace std;

#define BLOB_COUNT 20 /* Number of Blob particles in the world */
#define PLATFORM_COUNT 6 /* Number of Platform particles in the world */
#define NUMBER_OF_QUADS 4 /* Number of quadrants in the world */


const Vector2 Vector2::GRAVITY = Vector2(0,-9.81); 
vector<Particle*>  particlesQuad[NUMBER_OF_QUADS];  /*An array of vectors to hold Particle pointers for each quad*/
Particle* particlesQuadA[NUMBER_OF_QUADS][BLOB_COUNT];  /*A 2D array of Particle pointers for each quad and Number of Blob particles*/


/**
 * Platforms are two dimensional: lines on which the 
 * particles can rest. Platforms are also contact generators for the physics.
 */

class Platform : public ParticleContactGenerator
{
public:
    Vector2 start;
    Vector2 end;
    /**
     * Holds a pointer to the particles we're checking for collisions with. 
     */
    Particle *particle;

    virtual unsigned addContact(
        ParticleContact *contact, 
        unsigned limit
        ) const;
};


// ParticleCollision class - Inherits from ParticleContactGenerator
class ParticleCollision : public ParticleContactGenerator {
public:
    // Pointer to a particle involved in collision 
    Particle* particle = nullptr;

    // Number of blobs involved in the collision
    int blob_num;

    //Constructor: It helps to introduce particles to the particle world
    ParticleCollision(int blob_count) {
        blob_num = blob_count;
    }
    // Generates a contact between particles if a collision occurs
    virtual unsigned addContact(
        ParticleContact* contact,
        unsigned limit
    ) const;

    // Checks whether collision occurs between two particles
    bool checkCollision(Particle& particle1, Particle& particle2) const;
};

unsigned ParticleCollision::addContact(ParticleContact* contact,
    unsigned limit) const {
    const static float restitution = 1.0f; // Restitution for collision response
    unsigned used = 0; // tracking the total number of contacts generated

    // Loop through each quad of particles
    int k = 0;
    while (k < NUMBER_OF_QUADS) {
        if (particlesQuad[k].size() > 1) { // Checking if there are at least two particles in the quad
            for (int i = 0; i < particlesQuad[k].size() - 1; i++) {
                for (int j = i + 1; j < particlesQuad[k].size(); j++) {         
                    if (checkCollision(*particlesQuad[k][i], *particlesQuad[k][j])) { // Check for collision between the particles    

                        
                        Vector2 toParticle = particlesQuad[k][i]->getPosition() - particlesQuad[k][j]->getPosition(); // Calculating the distance between the vectors and storing it into vector
                        contact->contactNormal = toParticle.unit(); //calculating the unit vector (unit vector is also the direction)
                        contact->restitution = restitution; // setting the resitution from the above variable

                        // Stroing the checked particles in the contact
                        contact->particle[0] = particlesQuad[k][i];
                        contact->particle[1] = particlesQuad[k][j];

                        contact->penetration = (particlesQuad[k][i]->getRadius() + particlesQuad[k][j]->getRadius()) - toParticle.magnitude(); //Calculating the particles overlaps

                        //Moving to the next contact after 
                        used++;
                        contact++;
                    }
                }
            }
        }
        k++;
    }
    return used; // Return the number of generated contacts
}


bool ParticleCollision::checkCollision(Particle& particle1, Particle& particle2) const {

    Vector2 position = particle1.getPosition(); // Position of the first particle as a vector
    float radius = particle1.getRadius(); // Radius of the first particle

    Vector2 position_2 = particle2.getPosition(); // Position of the second particle as a vector
    float radius_2 = particle2.getRadius(); // Radius of the second particle

    float total_radius = radius + radius_2;  // total radius of the two particles
    float distance_between = (position_2 - position).magnitude(); // converting the distance from vector to numbers
    
    // Check if the distance is greater than the sum of radiuses
    if (distance_between > total_radius) return false;
    else return true;
}


unsigned Platform::addContact(ParticleContact *contact, 
                              unsigned limit) const
{
	const static float restitution = 1.0f;
	unsigned used = 0;
    for (int i = 0; i < BLOB_COUNT; i++) {
        // Check for penetration
        Vector2 toParticle = particle[i].getPosition() - start;
        Vector2 lineDirection = end - start;

        float projected = toParticle * lineDirection;
        float platformSqLength = lineDirection.squareMagnitude();
        float squareRadius = particle[i].getRadius() * particle[i].getRadius();;

        if (projected <= 0)
        {

            // The blob is nearest to the start point
            if (toParticle.squareMagnitude() < squareRadius)
            {
                // We have a collision
                contact->contactNormal = toParticle.unit();
                contact->restitution = restitution;
                contact->particle[0] = particle + i;
                contact->particle[1] = 0;
                contact->penetration = particle[i].getRadius() - toParticle.magnitude();
                used++;
                contact++;
            }

        }
        else if (projected >= platformSqLength)
        {
            // The blob is nearest to the end point
            toParticle = particle[i].getPosition() - end;
            if (toParticle.squareMagnitude() < squareRadius)
            {
                // We have a collision
                contact->contactNormal = toParticle.unit();
                contact->restitution = restitution;
                contact->particle[0] = particle+i;
                contact->particle[1] = 0;
                contact->penetration = particle[i].getRadius() - toParticle.magnitude();
                used++;
                contact++;
            }
        }
        else
        {
            // the blob is nearest to the middle.
            float distanceToPlatform = toParticle.squareMagnitude() - projected * projected / platformSqLength;
            if (distanceToPlatform < squareRadius)
            {
                // We have a collision
                Vector2 closestPoint = start + lineDirection * (projected / platformSqLength);

                contact->contactNormal = (particle[i].getPosition() - closestPoint).unit();
                contact->restitution = restitution;
                contact->particle[0] = particle+i;
                contact->particle[1] = 0;
                contact->penetration = particle[i].getRadius() - sqrt(distanceToPlatform);
                used++;
                contact++;
            }
        }
    }
    return used;
}


class BlobDemo : public Application
{
    Particle *blobs;

    Platform *platform;

    ParticleCollision* particleCol;

    ParticleWorld world;

public:
    /** Creates a new demo object. */
    BlobDemo();
    virtual ~BlobDemo();

    /** Returns the window title for the demo. */
    virtual const char* getTitle();

    /** Display the particles. */
    virtual void display();

    /** Update the particle positions. */
    virtual void update();
    Platform* buildPlatform();
	
};

// Method definitions
BlobDemo::BlobDemo():world(20000, 10000)
{
 
    
	width = 400; height = 400; 
	nRange = 100.0;

    blobs = new Particle[BLOB_COUNT]; // array of Particle objects with size BLOB_COUNT
    platform = buildPlatform(); // calling the build platform and storing the platform here
    
    particleCol = new ParticleCollision(BLOB_COUNT); // a new instance of the ParticleCollision class with how many blobs there are in the world
    particleCol->particle = blobs; // particle pointing to the blobs array
    world.getContactGenerators().push_back(particleCol); // adding the particle collision to the world 
    

    for (int i = 0; i < PLATFORM_COUNT; i++) {
        platform[i].particle = blobs;
        world.getContactGenerators().push_back(platform + i);
    }

    for (int i = 0; i < BLOB_COUNT; i++) {
        // generating some values for x,y in between [-50,50]
        float x = 100 / 2 + (rand() % 101 - 100); 
        float y = 100 / 2 + (rand() % 101 - 100);
        blobs[i].setPosition(x, y); //setting the value
        blobs[i].setRadius(5);
        blobs[i].setVelocity(0, 0);
        blobs[i].setDamping(0.9);
        blobs[i].setAcceleration(Vector2::GRAVITY * 5.0f * (i + 1));
        blobs[i].setMass(100.0f);
        blobs[i].clearAccumulator();
        world.getParticles().push_back(blobs + i);
    }

    for (int j = 0; j < BLOB_COUNT; j++) {
        Vector2 position = blobs[j].getPosition(); // position of the j-th particle in the 'blobs' array
        // checking the particles quadrant according to its position
        if ((position.x > -width) && (position.x < 0)) {
            if (position.y > 0)
                particlesQuad[0].push_back(blobs +j); // Top-left quadrant
            else
                particlesQuad[1].push_back(blobs + j); // Bottom-left quadrant
        }
        else {
            if (position.y > 0)
                particlesQuad[2].push_back(blobs + j); // Top-right quadrant
            else
                particlesQuad[3].push_back(blobs + j); // Bottom-right quadrant
        }
    }
  
   
}


BlobDemo::~BlobDemo()
{
    delete[] particlesQuadA;
    delete[] blobs;
    delete[] platform;
}

void BlobDemo::display()
{
  Application::display();
  for (int i = 0; i < PLATFORM_COUNT; i++) {
      const Vector2& p0 = platform[i].start;
      const Vector2& p1 = platform[i].end;

      glBegin(GL_LINES);
      glColor3f(0, 1, 1);
      glVertex2f(p0.x, p0.y);
      glVertex2f(p1.x, p1.y);
      glEnd();

      glColor3f(1, 0, 0);
  }
  for (int i = 0; i < BLOB_COUNT; i++) {
      const Vector2& p = blobs[i].getPosition();
      if (i % 4 == 0)
          glColor3ub(204, 255, 153);
      else if (i % 5 == 0)
          glColor3ub(51, 153, 255);
      else if (i % 3 == 0)
          glColor3ub(127, 0, 255);
      else if (i % 2 == 0)
          glColor3ub(51, 51, 255);
      else
          glColor3ub(255, 102, 102);
      glPushMatrix();
      glTranslatef(p.x, p.y, 0);
      glutSolidSphere(blobs[i].getRadius(), 12, 12);
      glPopMatrix();
  }
	glutSwapBuffers();
    
}
Platform* BlobDemo::buildPlatform() {


    float nrange = 100.0f;
    float margin = 0.95;

    platform = new Platform[PLATFORM_COUNT];  //An array of Platform objects


    // platforms 0 and 1 with specific start and end points
    platform[0].start = Vector2(-50.0, 20.0);
    platform[0].end = Vector2(0, -20.0);

    platform[1].start = Vector2(50.0, 20.0);
    platform[1].end = Vector2(0, -20.0);

    // setting the boundary around the range with a margin
    platform[2].start = Vector2(-nRange * margin, nRange * margin);
    platform[2].end = Vector2(nRange * margin, nRange * margin);

    platform[3].start = Vector2(-nRange * margin, -nRange * margin);
    platform[3].end = Vector2(nRange * margin, -nRange * margin);

    platform[4].start = Vector2(nRange * margin, -nRange * margin);
    platform[4].end = Vector2(nRange * margin, nRange * margin);

    platform[5].start = Vector2(-nRange * margin, nRange * margin);
    platform[5].end = Vector2(-nRange * margin, -nRange * margin);

    return platform;
}


void BlobDemo::update()
{
    // Recenter the axes
	float duration = timeinterval/1000;
    // Run the simulation
    world.runPhysics(duration);


    for (int i = 0; i < 4; i++)
        particlesQuad[i].clear(); // clearing the Quads


    // after every update
    for (int j = 0; j < BLOB_COUNT; j++) {
        Vector2 position = blobs[j].getPosition(); // position of the j-th particle in the 'blobs' array
        // checking the particles quadrant according to its position
        if ((position.x > -width) && (position.x < 0)) {
            if (position.y > 0)
                particlesQuad[0].push_back(blobs + j); // Top-left quadrant
            else
                particlesQuad[1].push_back(blobs + j); // Bottom-left quadrant
        }
        else {
            if (position.y > 0)
                particlesQuad[2].push_back(blobs + j); // Top-right quadrant
            else
                particlesQuad[3].push_back(blobs + j); // Bottom-right quadrant
        }
    }

    Application::update();
}

const char* BlobDemo::getTitle()
{
    return "Blob Demo";
}


Application* getApplication()
{
    return new BlobDemo();
}


// !-------------------- BRUTE FORCE -------------------------!

/*
unsigned ParticleCollision::addContact(ParticleContact* contact,
    unsigned limit) const {
    const static float restitution = 1.0f;
    unsigned used = 0;
    for (int i = 0; i < BLOB_COUNT; i++) {
        for (int j = i + 1; j < BLOB_COUNT; j++)
            if (checkCollision(particle[i], particle[j])) {
                Vector2 toParticle = particle[i].getPosition() - particle[j].getPosition();
                contact->contactNormal = toParticle.unit();
                contact->restitution = restitution;
                contact->particle[0] = &particle[i];
                contact->particle[1] = &particle[j];
                contact->penetration = (particle[i].getRadius() + particle[j].getRadius()) - toParticle.magnitude();
                //cout << "Penetration -> " << contact->penetration << endl;
                used++;
                contact++;
            }

    }
    return used;
}*/
