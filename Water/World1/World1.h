#import "cocos2d.h"
#import "Box2D.h"
#import "WorldSceneProtocol.h"

@class DLRenderTexture;

@interface World1 : CCLayer <WorldSceneProtocol>

-(void)addNewSpriteWithCoords:(CGPoint)p;

@end