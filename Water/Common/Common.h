//
//  Common.h
//  Water
//
//  Created by Alexey Tyurin on 28.02.13.
//
//

#import <UIKit/UIKit.h>
#import "Box2D.h"

#define DEBUG_DRAW 1

#define PTM_RATIO 32.0f

#define WORLD [Common world]
#define BATCH [Common batch]
#define SIZE [Common size]
#define PARTICLES_COUNT [Common getParticlesCount]

@interface Common : NSObject

+(b2World *)world;
+(CGSize)size;
+(CCSpriteBatchNode *)batch;
+(NSInteger)getParticlesCount;
+(void)setParticlesCount:(NSInteger)count_;

+(void)start;
+(void)createNextScene;
+(void)createPreviousScene;
+(void)createMenu;

+(void)draw;
+(void)processAccelometry:(UIAcceleration *)acceleration_;

@end
