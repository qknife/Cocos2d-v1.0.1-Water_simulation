//
//  Common.h
//  Water
//
//  Created by Alexey Tyurin on 28.02.13.
//
//

#import <UIKit/UIKit.h>
#import "Box2D.h"

#define DEBUG 1

#define PTM_RATIO 32.0f

#define WORLD [Common world]
#define PARTICLES_COUNT [Common getParticlesCount]

@interface Common : NSObject

+(b2World *)world;

+(void)createNextScene;
+(void)createPreviousScene;
+(void)createMenu;

@end
