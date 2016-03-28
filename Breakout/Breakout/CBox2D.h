//
//  CBox2D.h
//  Breakout
//
//  Created by Jack Tsai on 3/27/16.
//  Copyright © 2016 BCIT. All rights reserved.
//

#ifndef CBox2D_h
#define CBox2D_h

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>

@interface CBox2D : NSObject

-(id)init:(UIView *)view;
-(void)update:(float)dt;
-(void)render:(int)modelViewProjectionMatrixPtr;
-(void)handleTapFrom:(UITapGestureRecognizer *)recognizer;
-(void)handleLongPressFrom:(UILongPressGestureRecognizer *)recognizer;

@end

#endif /* CBox2D_h */
