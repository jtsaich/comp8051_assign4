//
//  ViewController.h
//  Breakout
//
//  Created by Jack Tsai on 3/27/16.
//  Copyright © 2016 BCIT. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>

@interface ViewController : GLKViewController
@property (weak, nonatomic) IBOutlet UILabel *tapToStart;
@property (weak, nonatomic) IBOutlet UILabel *instruction;
@end

