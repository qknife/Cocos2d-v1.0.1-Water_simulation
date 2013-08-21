//
//  Common.m
//  Water
//
//  Created by Alexey Tyurin on 28.02.13.
//
//

#import "Common.h"
#import "World1.h"
#import "World2.h"
#import "World3.h"
#import "DLRenderTexture.h"
#import "GLES-Render.h"

static const NSInteger maxSceneIndex = 2;

static CCScene<WorldSceneProtocol> *currentScene = nil;
static NSInteger currentSceneIndex = -1;
static NSArray *nameScene = [[NSArray alloc]initWithObjects:@"Box2D",@"SPH (Vavius)+Box2D",@"JD",nil];
static b2World *world;
static GLESDebugDraw *m_debugDraw;
static NSInteger particlesCount = 0;
static CCLabelTTF *particlesCountLabel = nil;
static CCSpriteBatchNode *batch = nil;
static DLRenderTexture *renderTexture = nil;
static CGSize size;

#define UNIT MH(1.0f)

#define UNIT_FONT (size.width / 100.0f * 0.15f)
#define TITLE_TEXT 32 * UNIT_FONT
#define MIDDLE_TEXT 28 * UNIT_FONT
#define SMALL_TEXT 24 * UNIT_FONT

@interface Common (Private)

+(void)cleanScene;
+(void)initializeScene;
+(void)createWorld;
+(void)createScene;

@end

@implementation Common

#pragma mark - accessors
+(b2World *)world
{
	return world;
}

+(CGSize)size
{
	return size;
}

+(CCScene<WorldSceneProtocol> *)getCurrentScene
{
	return currentScene;
}

+(CCSpriteBatchNode *)batch
{
	return batch;
}

+(NSInteger)getParticlesCount
{
	return particlesCount;
}

+(void)setParticlesCount:(NSInteger)count_
{
	if (count_ < 125)
	{
		return;
	}
	
	if (particlesCount < count_)
	{
		[currentScene particlesCountUp: count_ - particlesCount];
	}
	else
	{
		[currentScene particlesCountDown: particlesCount - count_];
	}
	
	particlesCount = count_;
	
	// change text label
	[particlesCountLabel setString:[NSString stringWithFormat:@"%d",particlesCount]];
}

#pragma mark - scenes
+(void)start
{
	size = CGSizeMake([[CCDirector sharedDirector] winSize].width * CC_CONTENT_SCALE_FACTOR(), [[CCDirector sharedDirector] winSize].height * CC_CONTENT_SCALE_FACTOR());
}

+(void)createNextScene
{
	currentSceneIndex++;
	if (currentSceneIndex > maxSceneIndex)
	{
		currentSceneIndex = maxSceneIndex;
	}
	else
	{
		[self createScene];
	}
}

+(void)createPreviousScene
{
	currentSceneIndex--;
	if (currentSceneIndex < 0)
	{
		currentSceneIndex = 0;
	}
	else
	{
		[self createScene];
	}
}

+(void)createScene
{
	// clean current scene
	[self cleanScene];
	
	// create world
	[self createWorld];	
	
	switch (currentSceneIndex)
	{
		case 0:
			currentScene = [World3 node];
			break;
		case 1:
			currentScene = [World1 node];
			break;
		case 2:
			currentScene = [World2 node];
			break;
	}
	
	// initilize scene
	[self initializeScene];
	
	// replace scene
	[[CCDirector sharedDirector] replaceScene:currentScene];
}

#pragma mark - private
+(void)createWorld
{
	/*** create world ***/
	b2Vec2 gravity;
	gravity.Set(0.0f, -10.0f);
	world = new b2World(gravity);
	world->SetAllowSleeping(true);
	world->SetContinuousPhysics(true);

#if 1 == DEBUG_DRAW
	/*** debug ***/
	m_debugDraw = new GLESDebugDraw(PTM_RATIO);
	world->SetDebugDraw(m_debugDraw);

	uint32 flags = 0;
	flags += b2Draw::e_shapeBit;
	m_debugDraw->SetFlags(flags);
#endif
	
	/*** ground ***/	
	// Define the ground body.
	b2BodyDef groundBodyDef;
	groundBodyDef.type = b2_staticBody;
	groundBodyDef.position.Set(0, 0); // bottom-left corner
	b2Body *groundBody = world->CreateBody(&groundBodyDef);
	
	// Define the ground box shape.
	b2FixtureDef gGroundFix;
	b2PolygonShape groundBox;
	gGroundFix.shape = &groundBox;
	
	// bottom
	groundBox.SetAsBox(100 * UNIT, UNIT / 5,b2Vec2(MW(50),MH(10)),0);
	groundBody->CreateFixture(&gGroundFix);
	
	// top
	groundBox.SetAsBox(100 * UNIT, UNIT / 5,b2Vec2(MW(50),MH(100)),0);
	groundBody->CreateFixture(&gGroundFix);

	// left
	groundBox.SetAsBox(UNIT / 5, 100 * UNIT,b2Vec2(MW(0),MH(50)),0);
	groundBody->CreateFixture(&gGroundFix);
	
	// right
	groundBox.SetAsBox(UNIT / 5, 100 * UNIT,b2Vec2(MW(100),MH(50)),0);
	groundBody->CreateFixture(&gGroundFix);
	
	/*** Static geometry ***/
    b2BodyDef bd;
    bd.type = b2_staticBody;
    bd.position.Set(0, 0);
    b2Body *obstacles = world->CreateBody(&bd);
	
    b2FixtureDef gFix;
	b2PolygonShape sd;
	gFix.shape = &sd;
	
	int const obstacles_count =3;
    
    
//	float data[obstacles_count][5] = {{1,3,49,49,-1.4f / 2},{5,1,40,47,0},{1,4,32,48,0},
//                                     {14,1,68,46,0},{1,11,53,53,0},{1,11,75,53,0}};
	
    float data[obstacles_count][5] ={{30,1.5f,50,30,0},{1.5f,10,25,38,0},{1.5f,10,75,38,0}
                                    };
    for (int i = 0; i < obstacles_count; i++)
	{
		sd.SetAsBox(data[i][0] * UNIT, data[i][1] * UNIT,b2Vec2(MW(data[i][2]),MH(data[i][3])),data[i][4]);
		obstacles->CreateFixture(&gFix);
	}
//	b2CircleShape cd;
//	gFix.shape = &cd;
//    cd.m_radius = 2.f * UNIT;
//    cd.m_p = b2Vec2(MW(37),MH(30));
//    obstacles->CreateFixture(&gFix);
//	cd.m_p = b2Vec2(MW(41),MH(33));
//    obstacles->CreateFixture(&gFix);
//	cd.m_p = b2Vec2(MW(46),MH(30));
//    obstacles->CreateFixture(&gFix);

}

+(void)initializeScene
{
	CGSize size = [CCDirector sharedDirector].winSize;
	
	// set up sprite
	batch = [[CCSpriteBatchNode batchNodeWithFile:@"drop.png" capacity:4000] retain];
	
	// render texture
	renderTexture = [DLRenderTexture renderTextureWithWidth:size.width height:size.height pixelFormat:kCCTexture2DPixelFormat_RGBA4444];
	renderTexture.position = ccp(size.width / 2, size.height / 2);
	[currentScene addChild:renderTexture];
	
	// create menu
	[self createMenu];
	
	// set sprites
	particlesCount = 0;
	[self setParticlesCount:500];//125
}

+(void)cleanScene
{
	if (nil == currentScene)
	{
		return;
	}
	
	if (world)
	{
		delete world;
		world = NULL;
	}
	
	if (m_debugDraw)
	{
		delete m_debugDraw;
		m_debugDraw = NULL;
	}
	
	if (batch)
	{
		[batch release];
	}
}

+(void)createMenu
{
	// ask director for the window size
	CGSize size = [[CCDirector sharedDirector] winSize];
	
	/*** name of scene ***/
	// create and initialize a Label
	CCLabelTTF *name_scene = [CCLabelTTF labelWithString:[nameScene objectAtIndex:currentSceneIndex] fontName:@"Marker Felt" fontSize:TITLE_TEXT];
	// position the label on the center of the screen
	name_scene.anchorPoint = ccp(0.5f, 0);
	name_scene.position =  ccp(size.width / 2 , size.height / 10 - name_scene.contentSize.height);
	// add the label as a child to this Layer
	[currentScene addChild: name_scene];
	
	/*** count of particles ***/
	// create and initialize a Label
	particlesCountLabel = [CCLabelTTF labelWithString:[NSString stringWithFormat:@"%d",particlesCount] fontName:@"Marker Felt" fontSize:SMALL_TEXT];
	// position the label on the center of the screen
	particlesCountLabel.anchorPoint = ccp(0, 0);
	particlesCountLabel.position =  ccp(0 , size.height / 10 - particlesCountLabel.contentSize.height);
	// add the label as a child to this Layer
	[currentScene addChild: particlesCountLabel];
	
	/*** switchers for scene ***/
	[CCMenuItemFont setFontSize:MIDDLE_TEXT];
	// Menu Item using blocks
	CCMenuItem *previous_scene = [CCMenuItemFont itemFromString:@"Previous" block:^(id sender)
								  {
									  [self createPreviousScene];
								  }];
	// Menu Item using blocks
	CCMenuItem *next_scene = [CCMenuItemFont itemFromString:@"Next" block:^(id sender)
							  {
								  [self createNextScene];
							  }];
	
	CCMenu *menu = [CCMenu menuWithItems:previous_scene, next_scene, nil];
	[menu alignItemsHorizontallyWithPadding:20];
	[menu setPosition:ccp(size.width/2, name_scene.position.y - MIDDLE_TEXT)];
	// Add the menu to the layer
	[currentScene addChild:menu];
	
	/*** switchers to particles count ***/
	// Menu Item using blocks
	CCMenuItem *count_up = [CCMenuItemFont itemFromString:@"*2" block:^(id sender)
								  {
									  [self setParticlesCount: particlesCount * 2];
								  }];
	// Menu Item using blocks
	CCMenuItem *count_down = [CCMenuItemFont itemFromString:@"/2" block:^(id sender)
							  {
								  [self setParticlesCount: particlesCount / 2];
							  }];
	
	
	menu = [CCMenu menuWithItems:count_up, count_down, nil];
	[menu alignItemsVerticallyWithPadding:10];
	[menu setPosition:ccp(size.width - MIDDLE_TEXT, name_scene.position.y - MIDDLE_TEXT / 2)];
	// Add the menu to the layer
	[currentScene addChild:menu];
}

#pragma mark - process scene
+(void)draw
{
	glDisable(GL_ALPHA_TEST);
	
    [renderTexture beginWithClear:0 g:0 b:0 a:0];
    [batch visit];
    [renderTexture end];

#if 1 == DEBUG_DRAW
	glPushMatrix();
	glDisable(GL_TEXTURE_2D);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	
	WORLD->DrawDebugData();
	
    glEnable(GL_TEXTURE_2D);
    glEnableClientState(GL_COLOR_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glPopMatrix();
#endif
}

+(void)processAccelometry:(UIAcceleration *)acceleration_
{
	static float prevX=0, prevY=0;
    
	//#define kFilterFactor 0.05f
#define kFilterFactor 1.0f	// don't use filter. the code is here just as an example
                           //Users/kvv_hacker/projects/Water/Water/Common/Common.mm
	float accelX = (float) acceleration_.x * kFilterFactor + (1- kFilterFactor)*prevX;
	float accelY = (float) acceleration_.y * kFilterFactor + (1- kFilterFactor)*prevY;
    
	prevX = accelX;
	prevY = accelY;
    
	// accelerometer values are in "Portrait" mode. Change them to Landscape left
	// multiply the gravity by 15
	b2Vec2 gravity(accelX * 15, accelY * 15);
    
	world->SetGravity(gravity);
}

@end
