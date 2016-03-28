//
//  CBox2D.mm
//  Breakout
//
//  Created by Jack Tsai on 3/27/16.
//  Copyright © 2016 BCIT. All rights reserved.
//

#include <Box2D/Box2D.h>
#include "CBox2D.h"
#include <OpenGLES/ES2/glext.h>
#include <stdio.h>

#define BUFFER_OFFSET(i) ((char *)NULL + (i))

#define PADDLE_WIDTH		100.0f
#define PADDLE_HEIGHT		20.0f
#define PADDLE_VELOCITY      500000.0f
#define BALL_RADIUS			15.0f
#define BALL_VELOCITY		100000.0f
#define BALL_SPHERE_SEGS	128
#define BRICK_COUNT         32

const float MAX_TIMESTEP = 1.0f/60.0f;
const int NUM_VEL_ITERATIONS = 10;
const int NUM_POS_ITERATIONS = 3;

#pragma mark - Box2D contact listener class
class CContactListener : public b2ContactListener {
public:
    void BeginContact(b2Contact *contact) {};
    void EndContact(b2Contact *contact) {};
    void PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {
        b2WorldManifold worldManifold;
        contact->GetWorldManifold(&worldManifold);
        b2PointState state1[2], state2[2];
        b2GetPointStates(state2, state2, oldManifold, contact->GetManifold());
        if (state2[0] == b2_addState) {
            b2Body *bodyA = contact->GetFixtureA()->GetBody();
            int i = 0;
            CBox2D *parentObj = (__bridge CBox2D*)(bodyA->GetUserData());
            
            NSLog(@"HIT");
        }
    }
    void PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {};
};

#pragma mark - CBox2D
@interface CBox2D () {
    int _width;
    int _height;
    b2Vec2 _paddlePosition;
    b2Vec2 _ballPosition;
    b2Vec2 _brickSize;
    b2Vec2 _bricksPosition[BRICK_COUNT];
    
    // variables for Box2D
    b2Vec2 *_gravity;
    b2World *_world;
    b2Body *_thePaddle, *_theBall;
    b2Body *_bricks[32];
    CContactListener *_contactListener;
    
    // variables for drawing
    GLuint paddleVertexArray, ballVertexArray, brickVertexArray;
    
    GLfloat _paddleVertexData[6 * 3];
    GLfloat _paddleColorData[6 * 3 * 3];
    GLKMatrix4 _paddleModelViewProjectionMatrix;
    
    GLfloat _ballVertexData[(BALL_SPHERE_SEGS+2) * 3];
    GLfloat _ballColorData[(BALL_SPHERE_SEGS+2) * 3 * 3];
    GLKMatrix4 _ballModelViewProjectionMatrix;
    
    GLfloat _brickVertexData[6 * 3];
    GLfloat _brickColorData[6 * 3 * 3];
    GLKMatrix4 _brickModelViewProjectionMatrix[BRICK_COUNT];
    
    // variables for game
    BOOL _brickHit[BRICK_COUNT];
}

@end

@implementation CBox2D

- (id)init:(UIView *)view {
    if (self = [super init]) {
        _width = view.frame.size.width;
        _height = view.frame.size.height;
        _paddlePosition = b2Vec2(_width / 2, 50);
        _ballPosition = b2Vec2(_width / 2, 150);
        
        _gravity = new b2Vec2(0.0f, 0.0f);
        _world = new b2World(*_gravity);
        
        _contactListener = new CContactListener();
        _world->SetContactListener(_contactListener);
        
        [self initBoundaries];
        [self initPaddle];
        [self initBall];
        [self initBricks];
    }
    
    
    return self;
}

- (void)initBoundaries {
    b2BodyDef ceilingBodyDef;
    ceilingBodyDef.type = b2_staticBody;
    ceilingBodyDef.position.Set(0, 0);
    b2Body *boundaries = _world->CreateBody(&ceilingBodyDef);
    
    if (boundaries) {
        boundaries->SetUserData((__bridge void *)self);
        b2Vec2 vs[4];
        vs[0] = b2Vec2(0, 0);
        vs[1] = b2Vec2(0, _height);
        vs[2] = b2Vec2(_width, _height);
        vs[3] = b2Vec2(_width, 0);
        b2ChainShape chain;
        chain.CreateChain(vs, 4);
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &chain;
        fixtureDef.friction = 0.0f;
        boundaries->CreateFixture(&fixtureDef);
    }
}

- (void)initPaddle {
    // Box2D setup
    b2BodyDef paddleBodyDef;
    paddleBodyDef.type = b2_kinematicBody;
    paddleBodyDef.position.Set(_paddlePosition.x, _paddlePosition.y);
    _thePaddle = _world->CreateBody(&paddleBodyDef);
    
    if (_thePaddle) {
        _thePaddle->SetUserData((__bridge void *)self);
        _thePaddle->SetAwake(false);
        b2PolygonShape dynamicBox;
        dynamicBox.SetAsBox(PADDLE_WIDTH / 2, PADDLE_HEIGHT / 2);
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &dynamicBox;
        fixtureDef.density = 1.0f;
        fixtureDef.friction = 0.3f;
        fixtureDef.restitution = 1.0f;
        _thePaddle->CreateFixture(&fixtureDef);
    }
    
    // VBOs setup
    int k = 0;
    _paddleVertexData[k++] = - PADDLE_WIDTH/2;
    _paddleVertexData[k++] = PADDLE_HEIGHT/2;
    _paddleVertexData[k++] = 10;
    _paddleVertexData[k++] = PADDLE_WIDTH/2;
    _paddleVertexData[k++] = PADDLE_HEIGHT/2;
    _paddleVertexData[k++] = 10;
    _paddleVertexData[k++] = PADDLE_WIDTH/2;
    _paddleVertexData[k++] = - PADDLE_HEIGHT/2;
    _paddleVertexData[k++] = 10;
    _paddleVertexData[k++] = - PADDLE_WIDTH/2;
    _paddleVertexData[k++] = PADDLE_HEIGHT/2;
    _paddleVertexData[k++] = 10;
    _paddleVertexData[k++] = PADDLE_WIDTH/2;
    _paddleVertexData[k++] = - PADDLE_HEIGHT/2;
    _paddleVertexData[k++] = 10;
    _paddleVertexData[k++] = - PADDLE_WIDTH/2;
    _paddleVertexData[k++] = - PADDLE_HEIGHT/2;
    _paddleVertexData[k++] = 10;
    
    for (k=0; k<sizeof(_paddleColorData) / sizeof(GLfloat); k+=3)
    {
        _paddleColorData[k] = 1.0f;
    }
    
    glGenVertexArraysOES(1, &paddleVertexArray);
    glBindVertexArrayOES(paddleVertexArray);
    
    GLuint vertexBuffers[2];
    glGenBuffers(2, vertexBuffers);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(_paddleVertexData), _paddleVertexData, GL_STATIC_DRAW);
    
    glEnableVertexAttribArray(GLKVertexAttribPosition);
    glVertexAttribPointer(GLKVertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
    
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(_paddleColorData), _paddleColorData, GL_STATIC_DRAW);
    
    glEnableVertexAttribArray(GLKVertexAttribColor);
    glVertexAttribPointer(GLKVertexAttribColor, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
    
    glBindVertexArrayOES(0);
}

- (void)initBall {
    // Box2D setup
    b2BodyDef ballBodyDef;
    ballBodyDef.type = b2_dynamicBody;
    ballBodyDef.position.Set(_ballPosition.x, _ballPosition.y);
    _theBall = _world->CreateBody(&ballBodyDef);
    
    if (_theBall) {
        _theBall->SetUserData((__bridge void *)self);
        _theBall->SetAwake(false);
        b2CircleShape circle;
        circle.m_p.Set(0, 0);
        circle.m_radius = BALL_RADIUS;
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &circle;
        fixtureDef.density = 1.0f;
        fixtureDef.friction = 0.3f;
        fixtureDef.restitution = 1.0;
        _theBall->CreateFixture(&fixtureDef);
    }
    
    // VBOs setup
    int k = 0;
    _ballVertexData[k++] = 0;
    _ballVertexData[k++] = 0;
    _ballVertexData[k++] = 0;
    for (int n=0; n<=BALL_SPHERE_SEGS; n++) {
        float const t = 2*M_PI*(float)n/(float)BALL_SPHERE_SEGS;
        _ballVertexData[k++] = sin(t)*BALL_RADIUS;
        _ballVertexData[k++] = cos(t)*BALL_RADIUS;
        _ballVertexData[k++] = 0;
    }
    
    for (k=1; k<sizeof(_ballColorData) / sizeof(GLfloat); k+=3) {
        _ballColorData[k] = 1.0f;
    }
    
    glGenVertexArraysOES(1, &ballVertexArray);
    glBindVertexArrayOES(ballVertexArray);
    
    GLuint vertexBuffers[2];
    glGenBuffers(2, vertexBuffers);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(_ballVertexData), _ballVertexData, GL_STATIC_DRAW);
    
    glEnableVertexAttribArray(GLKVertexAttribPosition);
    glVertexAttribPointer(GLKVertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
    
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(_ballColorData), _ballColorData, GL_STATIC_DRAW);
    
    glEnableVertexAttribArray(GLKVertexAttribColor);
    glVertexAttribPointer(GLKVertexAttribColor, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
    
    glBindVertexArrayOES(0);
}

- (void)initBricks {
    int margin = 40;
    int brickWidth = ((_width - margin * 2) / 8);
    int brickHeight = 20;
    int brickPadding = 4;
    
    for (int i = 0; i < BRICK_COUNT; i++) {
        _bricksPosition[i] = b2Vec2(margin + (i % 8 * (brickWidth + brickPadding)), _height - margin - (i / 8 * (brickHeight + brickPadding)) - (brickHeight/2));
        
        // Box2D setup
        b2BodyDef brickBodyDef;
        brickBodyDef.type = b2_staticBody;
        brickBodyDef.position.Set(_bricksPosition[i].x, _bricksPosition[i].y);
        b2Body *brick = _world->CreateBody(&brickBodyDef);
        
        if (brick) {
            brick->SetUserData((__bridge void *)self);
            brick->SetAwake(false);
            b2PolygonShape dynamicBox;
            dynamicBox.SetAsBox(40 / 2, 20 / 2);
            b2FixtureDef fixtureDef;
            fixtureDef.shape = &dynamicBox;
            fixtureDef.density = 1.0f;
            fixtureDef.friction = 0.3f;
            fixtureDef.restitution = 1.0f;
            brick->CreateFixture(&fixtureDef);
        }
        
        _bricks[i] = brick;
    }
    
    // VBOs setup
    int k = 0;
    _brickVertexData[k++] = - brickWidth/2;
    _brickVertexData[k++] = brickHeight/2;
    _brickVertexData[k++] = 10;
    _brickVertexData[k++] = brickWidth/2;
    _brickVertexData[k++] = brickHeight/2;
    _brickVertexData[k++] = 10;
    _brickVertexData[k++] = brickWidth/2;
    _brickVertexData[k++] = - brickHeight/2;
    _brickVertexData[k++] = 10;
    _brickVertexData[k++] = - brickWidth/2;
    _brickVertexData[k++] = brickHeight/2;
    _brickVertexData[k++] = 10;
    _brickVertexData[k++] = brickWidth/2;
    _brickVertexData[k++] = - brickHeight/2;
    _brickVertexData[k++] = 10;
    _brickVertexData[k++] = - brickWidth/2;
    _brickVertexData[k++] = - brickHeight/2;
    _brickVertexData[k++] = 10;
    
    for (k=2; k<sizeof(_brickColorData) / sizeof(GLfloat); k+=3)
    {
        _brickColorData[k] = 1.0f;
    }
    
    glGenVertexArraysOES(1, &brickVertexArray);
    glBindVertexArrayOES(brickVertexArray);
    
    GLuint vertexBuffers[2];
    glGenBuffers(2, vertexBuffers);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(_brickVertexData), _brickVertexData, GL_STATIC_DRAW);
    
    glEnableVertexAttribArray(GLKVertexAttribPosition);
    glVertexAttribPointer(GLKVertexAttribPosition, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
    
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(_brickColorData), _brickColorData, GL_STATIC_DRAW);
    
    glEnableVertexAttribArray(GLKVertexAttribColor);
    glVertexAttribPointer(GLKVertexAttribColor, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
    
    glBindVertexArrayOES(0);
}

- (void)update:(float)dt {
    if (_world)
    {
        while (dt >= MAX_TIMESTEP)
        {
            _world->Step(MAX_TIMESTEP, NUM_VEL_ITERATIONS, NUM_POS_ITERATIONS);
            dt -= MAX_TIMESTEP;
        }
        
        if (dt > 0.0f)
        {
            _world->Step(dt, NUM_VEL_ITERATIONS, NUM_POS_ITERATIONS);
        }
    }
    
    GLKMatrix4 projectionMatrix = GLKMatrix4MakeOrtho(0, _width, 0, _height, -10, 100);
    GLKMatrix4 modelViewMatrix = GLKMatrix4MakeTranslation(_thePaddle->GetPosition().x, _thePaddle->GetPosition().y, 0);
    _paddleModelViewProjectionMatrix = GLKMatrix4Multiply(projectionMatrix, modelViewMatrix);
    
    modelViewMatrix = GLKMatrix4MakeTranslation(_theBall->GetPosition().x, _theBall->GetPosition().y, 0);
    _ballModelViewProjectionMatrix = GLKMatrix4Multiply(projectionMatrix, modelViewMatrix);
    
    for (int i = 0; i < BRICK_COUNT; i++) {
        modelViewMatrix = GLKMatrix4MakeTranslation(_bricks[i]->GetPosition().x, _bricks[i]->GetPosition().y, 0);
        _brickModelViewProjectionMatrix[i] = GLKMatrix4Multiply(projectionMatrix, modelViewMatrix);
        
    }
}

- (void)render:(int)modelViewProjectionMatrixPtr {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glUniformMatrix4fv(modelViewProjectionMatrixPtr, 1, 0, _paddleModelViewProjectionMatrix.m);
    glBindVertexArrayOES(paddleVertexArray);
    if (_thePaddle && sizeof(_paddleVertexData) > 0)
        glDrawArrays(GL_TRIANGLES, 0, sizeof(_paddleVertexData) / sizeof(GLfloat) / 3);
        
        
    glUniformMatrix4fv(modelViewProjectionMatrixPtr, 1, 0, _ballModelViewProjectionMatrix.m);
    glBindVertexArrayOES(ballVertexArray);
    if (_theBall && sizeof(_ballVertexData) > 0)
        glDrawArrays(GL_TRIANGLE_FAN, 0, sizeof(_ballVertexData) / sizeof(GLfloat) / 3);
        
    for (int i = 0; i < BRICK_COUNT; i++) {
        glUniformMatrix4fv(modelViewProjectionMatrixPtr, 1, 0, _brickModelViewProjectionMatrix[i].m);
        glBindVertexArrayOES(brickVertexArray);
        if (_bricks[i] && sizeof(_brickVertexData) > 0)
            glDrawArrays(GL_TRIANGLES, 0, sizeof(_brickVertexData) / sizeof(GLfloat) / 3);
    }
}

- (void)handleTapFrom:(UITapGestureRecognizer *)recognizer {
    if (_theBall->GetLinearVelocity().y == 0) {
        _theBall->SetActive(true);
        _theBall->SetLinearVelocity(b2Vec2(0, -200));
    }
}

- (void)handleLongPressFrom:(UILongPressGestureRecognizer *)recognizer {
    if (UIGestureRecognizerStateBegan == recognizer.state) {
        CGPoint touchLocation = [recognizer locationInView:recognizer.view];
        if (touchLocation.x < recognizer.view.frame.size.width / 2) {
            _thePaddle->SetLinearVelocity(b2Vec2(-PADDLE_VELOCITY, 0));
        } else {
            _thePaddle->SetLinearVelocity(b2Vec2(PADDLE_VELOCITY, 0));
        }
    } else if (UIGestureRecognizerStateEnded == recognizer.state) {
        _thePaddle->SetLinearVelocity(b2Vec2(0, 0));
    }
}

@end
