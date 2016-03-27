//
//  CBox2D.m
//  MyGLGame
//
//  Created by Borna Noureddin on 2015-03-17.
//  Copyright (c) 2015 BCIT. All rights reserved.
//

#include <Box2D/Box2D.h>
#include "CBox2D.h"
#include <OpenGLES/ES2/glext.h>
#include <stdio.h>

#define BUFFER_OFFSET(i) ((char *)NULL + (i))
//#define LOG_TO_CONSOLE



#pragma mark - Brick and ball physics parameters

#define BRICK_A_POS_X			400
#define BRICK_A_POS_Y			550
#define BRICK_A_WIDTH			120.0f
#define BRICK_A_HEIGHT		10.0f
#define BRICK_A_WAIT			1.5f

#define BRICK_B_POS_X			400
#define BRICK_B_POS_Y			50
#define BRICK_B_WIDTH			120.0f
#define BRICK_B_HEIGHT		10.0f
#define BRICK_B_WAIT			1.5f


#define BALL_POS_X			400
#define BALL_POS_Y			300
#define BALL_RADIUS			15.0f
#define BALL_VELOCITY		100000.0f
#define BALL_SPHERE_SEGS	16

const float MAX_TIMESTEP = 1.0f/60.0f;
const int NUM_VEL_ITERATIONS = 10;
const int NUM_POS_ITERATIONS = 3;


#pragma mark - Box2D contact listener class

class CContactListener : public b2ContactListener
{
public:
    void BeginContact(b2Contact* contact) {};
    void EndContact(b2Contact* contact) {};
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
    {
        b2WorldManifold worldManifold;
        contact->GetWorldManifold(&worldManifold);
        b2PointState state1[2], state2[2];
        b2GetPointStates(state1, state2, oldManifold, contact->GetManifold());
        if (state2[0] == b2_addState)
        {
            b2Body* bodyA = contact->GetFixtureA()->GetBody();
            CBox2D *parentObj = (__bridge CBox2D *)(bodyA->GetUserData());
            [parentObj RegisterHit];
        }
    }
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {};
};


#pragma mark - CBox2D

@interface CBox2D ()
{
    // Box2D-specific objects
    b2Vec2 *gravity;
    b2World *world;
    b2BodyDef *groundBodyDef;
    b2Body *groundBody;
    b2PolygonShape *groundBox;
    b2Body *theBrick_A, *theBrick_B, *theBall;
    CContactListener *contactListener;
    
    // GL-specific variables
    GLuint brickVertexArray[2], ballVertexArray;
    int numBrickVerts, numBallVerts;
    GLKMatrix4 modelViewProjectionMatrix;

    bool ballHitBrick;
    bool ballLaunched;
    float totalElapsedTime;
}
@end

@implementation CBox2D

- (instancetype)init
{
    self = [super init];
    if (self) {
        
        // setting gracity in 2d environment
        gravity = new b2Vec2(0.0f, 0.0f);
        // create a wrold
        world = new b2World(*gravity);
        
        // For HelloWorld
//        groundBodyDef = NULL;
//        groundBody = NULL;
//        groundBox = NULL;

        // For brick & ball sample
        contactListener = new CContactListener();
        world->SetContactListener(contactListener);
        
        
        // A body definition holds all the data needed to construct a rigid body.
        b2BodyDef AbrickBodyDef;
        // dynamic: positive mass, non-zero velocity determined by forces, moved by solver
        AbrickBodyDef.type = b2_dynamicBody;
        // setting the position
        AbrickBodyDef.position.Set(BRICK_A_POS_X, BRICK_A_POS_Y);
        // pass the definition to the body A
        theBrick_A = world->CreateBody(&AbrickBodyDef);
        
        if (theBrick_A)
        {
            // useing the data
            theBrick_A->SetUserData((__bridge void *)self);
            // availavle the collision ???
            theBrick_A->SetAwake(false);
            // setting collision boundary
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(BRICK_A_WIDTH/2, BRICK_A_HEIGHT/2);
            // setting physical property
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicBox;
            fixtureDef.density = 1000.0f;
            fixtureDef.friction = 0.0f;
            fixtureDef.restitution = 10.0f;
            
            // create a entity ???
            theBrick_A->CreateFixture(&fixtureDef);
        }
        
        b2BodyDef BbrickBodyDef;
        BbrickBodyDef.type = b2_dynamicBody;
        BbrickBodyDef.position.Set(BRICK_B_POS_X, BRICK_B_POS_Y);
        theBrick_B = world->CreateBody(&BbrickBodyDef);
        
        if (theBrick_B)
        {

            theBrick_B->SetUserData((__bridge void *)self);
            theBrick_B->SetAwake(false);
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(BRICK_B_WIDTH/2, BRICK_B_HEIGHT/2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicBox;
            fixtureDef.density = 1000.0f;
            fixtureDef.friction = 0.0f;
            fixtureDef.restitution = 10.0f;
            theBrick_B->CreateFixture(&fixtureDef);
        }
        
        
        b2BodyDef ballBodyDef;
        ballBodyDef.type = b2_dynamicBody;
        ballBodyDef.position.Set(BALL_POS_X, BALL_POS_Y);
        theBall = world->CreateBody(&ballBodyDef);
        
        if (theBall)
        {
            theBall->SetUserData((__bridge void *)self);
            theBall->SetAwake(false);
            b2CircleShape circle;
            circle.m_p.Set(0, 0);
            circle.m_radius = BALL_RADIUS;
            b2FixtureDef circleFixtureDef;
            circleFixtureDef.shape = &circle;
            circleFixtureDef.density = 0.0f;
            circleFixtureDef.friction = 0.0f;
            circleFixtureDef.restitution = 1.0f;
            theBall->CreateFixture(&circleFixtureDef);
        }
        
        totalElapsedTime = 0;
        ballHitBrick = false;
        ballLaunched = false;
    }
    return self;
}

- (void)dealloc
{
    if (gravity) delete gravity;
    if (world) delete world;
    if (groundBodyDef) delete groundBodyDef;
    if (groundBox) delete groundBox;
    if (contactListener) delete contactListener;
}

-(void)Update:(float)elapsedTime
{
    [self theBrickAMove];
    
    if(theBall->GetPosition().x <= 0 - BALL_RADIUS
       || theBall->GetPosition().x >= 800 + BALL_RADIUS
       || theBall->GetPosition().y <= 0 - BALL_RADIUS
       || theBall->GetPosition().y >= 800 + BALL_RADIUS)
    {
        [self BallReset];
    }
    
    if (ballLaunched)
    {
        // apply a impulse with a linear velocity
        theBall->ApplyLinearImpulse(b2Vec2(0, -BALL_VELOCITY), theBall->GetPosition(), true);
        theBall->SetActive(true);
        
#ifdef LOG_TO_CONSOLE
        NSLog(@"Applying impulse %f to ball\n", BALL_VELOCITY);
#endif
        ballLaunched = false;
    }
    
    totalElapsedTime += elapsedTime;
    if ((totalElapsedTime > BRICK_A_WAIT) && theBrick_A)
        theBrick_A->SetAwake(true);
    
    if ((totalElapsedTime > BRICK_B_WAIT) && theBrick_B)
        theBrick_B->SetAwake(true);

    if (world)
    {
        while (elapsedTime >= MAX_TIMESTEP)
        {
            world->Step(MAX_TIMESTEP, NUM_VEL_ITERATIONS, NUM_POS_ITERATIONS);
            elapsedTime -= MAX_TIMESTEP;
        }
        
        if (elapsedTime > 0.0f)
        {
            world->Step(elapsedTime, NUM_VEL_ITERATIONS, NUM_POS_ITERATIONS);
        }
    }

    // all these about draw rectangle with vertext just like as 1,2,3. no more comments for this.
    glEnable(GL_DEPTH_TEST);

    if (theBrick_A)
    {
        glGenVertexArraysOES(1, &brickVertexArray[0]);
        glBindVertexArrayOES(brickVertexArray[0]);
        
        GLuint vertexBuffers[2];
        glGenBuffers(2, vertexBuffers);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[0]);
        GLfloat vertPos[18];
        int k = 0;
        numBrickVerts = 0;
        vertPos[k++] = theBrick_A->GetPosition().x - BRICK_A_WIDTH/2;
        vertPos[k++] = theBrick_A->GetPosition().y + BRICK_A_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = theBrick_A->GetPosition().x + BRICK_A_WIDTH/2;
        vertPos[k++] = theBrick_A->GetPosition().y + BRICK_A_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = theBrick_A->GetPosition().x + BRICK_A_WIDTH/2;
        vertPos[k++] = theBrick_A->GetPosition().y - BRICK_A_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = theBrick_A->GetPosition().x - BRICK_A_WIDTH/2;
        vertPos[k++] = theBrick_A->GetPosition().y + BRICK_A_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = theBrick_A->GetPosition().x + BRICK_A_WIDTH/2;
        vertPos[k++] = theBrick_A->GetPosition().y - BRICK_A_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = theBrick_A->GetPosition().x - BRICK_A_WIDTH/2;
        vertPos[k++] = theBrick_A->GetPosition().y - BRICK_A_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertPos), vertPos, GL_STATIC_DRAW);
        glEnableVertexAttribArray(VertexAttribPosition);
        glVertexAttribPointer(VertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
        
        GLfloat vertCol[18];
        for (k=0; k<numBrickVerts*3; k+=3)
        {
            vertCol[k] = 1.0f;
            vertCol[k+1] = 0.0f;
            vertCol[k+2] = 0.0f;
        }
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[1]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertCol), vertCol, GL_STATIC_DRAW);
        glEnableVertexAttribArray(VertexAttribColor);
        glVertexAttribPointer(VertexAttribColor, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
        
        glBindVertexArrayOES(0);
    }
    
    if (theBrick_B)
    {
        glGenVertexArraysOES(1, &brickVertexArray[1]);
        glBindVertexArrayOES(brickVertexArray[1]);
        
        GLuint vertexBuffers[2];
        glGenBuffers(2, vertexBuffers);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[0]);
        GLfloat vertPos[18];
        int k = 0;
        numBrickVerts = 0;
        vertPos[k++] = theBrick_B->GetPosition().x - BRICK_B_WIDTH/2;
        vertPos[k++] = theBrick_B->GetPosition().y + BRICK_B_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = theBrick_B->GetPosition().x + BRICK_B_WIDTH/2;
        vertPos[k++] = theBrick_B->GetPosition().y + BRICK_B_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = theBrick_B->GetPosition().x + BRICK_B_WIDTH/2;
        vertPos[k++] = theBrick_B->GetPosition().y - BRICK_B_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = theBrick_B->GetPosition().x - BRICK_B_WIDTH/2;
        vertPos[k++] = theBrick_B->GetPosition().y + BRICK_B_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = theBrick_B->GetPosition().x + BRICK_B_WIDTH/2;
        vertPos[k++] = theBrick_B->GetPosition().y - BRICK_B_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        vertPos[k++] = theBrick_B->GetPosition().x - BRICK_B_WIDTH/2;
        vertPos[k++] = theBrick_B->GetPosition().y - BRICK_B_HEIGHT/2;
        vertPos[k++] = 10;
        numBrickVerts++;
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertPos), vertPos, GL_STATIC_DRAW);
        glEnableVertexAttribArray(VertexAttribPosition);
        glVertexAttribPointer(VertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
        
        GLfloat vertCol[18];
        for (k=0; k<numBrickVerts*3; k+=3)
        {
            vertCol[k] = 1.0f;
            vertCol[k+1] = 0.0f;
            vertCol[k+2] = 0.0f;
        }
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[1]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertCol), vertCol, GL_STATIC_DRAW);
        glEnableVertexAttribArray(VertexAttribColor);
        glVertexAttribPointer(VertexAttribColor, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
        
        glBindVertexArrayOES(0);
    }

    
    if (theBall)
    {
        glGenVertexArraysOES(1, &ballVertexArray);
        glBindVertexArrayOES(ballVertexArray);
        
        GLuint vertexBuffers[2];
        glGenBuffers(2, vertexBuffers);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[0]);
        GLfloat vertPos[3*(BALL_SPHERE_SEGS+2)];
        int k = 0;
        vertPos[k++] = theBall->GetPosition().x;
        vertPos[k++] = theBall->GetPosition().y;
        vertPos[k++] = 0;
        numBallVerts = 1;
        for (int n=0; n<=BALL_SPHERE_SEGS; n++)
        {
            float const t = 2*M_PI*(float)n/(float)BALL_SPHERE_SEGS;
            vertPos[k++] = theBall->GetPosition().x + sin(t)*BALL_RADIUS;
            vertPos[k++] = theBall->GetPosition().y + cos(t)*BALL_RADIUS;
            vertPos[k++] = 0;
            numBallVerts++;
        }
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertPos), vertPos, GL_STATIC_DRAW);
        glEnableVertexAttribArray(VertexAttribPosition);
        glVertexAttribPointer(VertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
        
        GLfloat vertCol[numBallVerts*3];
        for (k=0; k<numBallVerts*3; k+=3)
        {
            vertCol[k] = 0.0f;
            vertCol[k+1] = 1.0f;
            vertCol[k+2] = 0.0f;
        }
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[1]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertCol), vertCol, GL_STATIC_DRAW);
        glEnableVertexAttribArray(VertexAttribColor);
        glVertexAttribPointer(VertexAttribColor, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
        
        glBindVertexArrayOES(0);
    }

    GLKMatrix4 projectionMatrix = GLKMatrix4MakeOrtho(0, 800, 0, 600, -10, 100);
    GLKMatrix4 modelViewMatrix = GLKMatrix4Identity;
    modelViewProjectionMatrix = GLKMatrix4Multiply(projectionMatrix, modelViewMatrix);
}

-(void)Render:(int)mvpMatPtr
{
#ifdef LOG_TO_CONSOLE
    if (theBall)
        printf("Ball: (%5.3f,%5.3f)\t",
               theBall->GetPosition().x, theBall->GetPosition().y);
    if (theBrick)
        printf("Brick: (%5.3f,%5.3f)",
               theBrick->GetPosition().x, theBrick->GetPosition().y);
    printf("\n");
#endif
    
    glClearColor(0, 0, 0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glUniformMatrix4fv(mvpMatPtr, 1, 0, modelViewProjectionMatrix.m);

    glBindVertexArrayOES(brickVertexArray[0]);
    if (theBrick_A && numBrickVerts > 0)
        glDrawArrays(GL_TRIANGLES, 0, numBrickVerts);
    
    glBindVertexArrayOES(brickVertexArray[1]);
    if (theBrick_B && numBrickVerts > 0)
        glDrawArrays(GL_TRIANGLES, 0, numBrickVerts);
    
    glBindVertexArrayOES(ballVertexArray);
    if (theBall && numBallVerts > 0)
        glDrawArrays(GL_LINE_LOOP, 0, numBallVerts);
}

-(void)RegisterHit
{
    ballHitBrick = true;
}

-(void)LaunchBall
{
    ballLaunched = true;
}

- (void)theBrickBMove:(float)x
{
    theBrick_B->SetTransform(b2Vec2(theBrick_B->GetPosition().x + x, BRICK_B_POS_Y), 0.0f);
}
- (void)theBrickAMove
{
    theBrick_A->SetTransform(b2Vec2(theBall->GetPosition().x, BRICK_A_POS_Y), 0.0f);
}

- (void)BallReset
{
    theBall->SetTransform(b2Vec2(BALL_POS_X, BALL_POS_Y), 0.0f);
}

//-(void)HelloWorld
//{
//    groundBodyDef = new b2BodyDef;
//    groundBodyDef->position.Set(0.0f, -10.0f);
//    groundBody = world->CreateBody(groundBodyDef);
//    groundBox = new b2PolygonShape;
//    groundBox->SetAsBox(50.0f, 10.0f);
//    
//    groundBody->CreateFixture(groundBox, 0.0f);
//    
//    // Define the dynamic body. We set its position and call the body factory.
//    b2BodyDef bodyDef;
//    bodyDef.type = b2_dynamicBody;
//    bodyDef.position.Set(0.0f, 4.0f);
//    b2Body* body = world->CreateBody(&bodyDef);
//    
//    // Define another box shape for our dynamic body.
//    b2PolygonShape dynamicBox;
//    dynamicBox.SetAsBox(1.0f, 1.0f);
//    
//    // Define the dynamic body fixture.
//    b2FixtureDef fixtureDef;
//    fixtureDef.shape = &dynamicBox;
//    
//    // Set the box density to be non-zero, so it will be dynamic.
//    fixtureDef.density = 1.0f;
//    
//    // Override the default friction.
//    fixtureDef.friction = 0.3f;
//    
//    // Add the shape to the body.
//    body->CreateFixture(&fixtureDef);
//    
//    // Prepare for simulation. Typically we use a time step of 1/60 of a
//    // second (60Hz) and 10 iterations. This provides a high quality simulation
//    // in most game scenarios.
//    float32 timeStep = 1.0f / 60.0f;
//    int32 velocityIterations = 6;
//    int32 positionIterations = 2;
//    
//    // This is our little game loop.
//    for (int32 i = 0; i < 60; ++i)
//    {
//        // Instruct the world to perform a single step of simulation.
//        // It is generally best to keep the time step and iterations fixed.
//        world->Step(timeStep, velocityIterations, positionIterations);
//        
//        // Now print the position and angle of the body.
//        b2Vec2 position = body->GetPosition();
//        float32 angle = body->GetAngle();
//        
//        printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
//    }
//}

@end
