#import "cocos2d.h"
#import "Box2D.h"
#import "GLES-Render.h"
#import "WorldSceneProtocol.h"

@class DLRenderTexture;

@interface World1 : CCLayer <WorldSceneProtocol>
{
    DLRenderTexture *renderTextureB;
    CCSpriteBatchNode *batch;
}

-(void)initializeScene;

// adds a new sprite at a given coordinate
-(void) addNewSpriteWithCoords:(CGPoint)p;

@end