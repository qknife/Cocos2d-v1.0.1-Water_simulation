#import "Common.h"
#import "WorldSceneProtocol.h"

@interface World1 : CCLayer <WorldSceneProtocol>

-(void)addNewSpriteWithCoords:(CGPoint)p;

@end