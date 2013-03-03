//
//  Common.m
//  Water
//
//  Created by Alexey Tyurin on 28.02.13.
//
//

#import "Common.h"
#import "World1.h"
#import "WorldSceneProtocol.h"
#import "DLRenderTexture.h"
#import "GLES-Render.h"

static const NSInteger maxSceneIndex = 2;
static const NSInteger labelFontSize = 28;

static CCScene<WorldSceneProtocol> *currentScene = nil;
static NSInteger currentSceneIndex = -1;
static NSArray *nameScene = [[NSArray alloc]initWithObjects:@"Box2D",@"SPH",@"JD",nil];
static b2World *world;
static GLESDebugDraw *m_debugDraw;
static NSInteger particlesCount = 125;
static CCLabelTTF *particlesCountLabel = nil;
static CCSpriteBatchNode *batch = nil;
static DLRenderTexture *renderTexture = nil;

@interface Common (Private)

+(void)cleanScene;
+(void)initializeScene;
+(void)createWorld;

@end

@implementation Common

#pragma mark - accessors
+(b2World *)world
{
	return world;
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
+(void)createNextScene
{
	// clean current scene
	[self cleanScene];
	
	// create world
	[self createWorld];
	
	currentSceneIndex++;
	switch (currentSceneIndex)
	{
		case 0:
			currentScene = [World1 node];
			break;
	}
	
	// initilize scene
	[self initializeScene];
	
	// replace scene
	[[CCDirector sharedDirector] replaceScene:[CCTransitionFade transitionWithDuration:0.5f scene:currentScene]];
}

+(void)createPreviousScene
{
	// clean current scene
	[self cleanScene];
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
	
	/*** debug ***/
	m_debugDraw = new GLESDebugDraw(PTM_RATIO);
	world->SetDebugDraw(m_debugDraw);

	uint32 flags = 0;
	flags += b2Draw::e_shapeBit;
	m_debugDraw->SetFlags(flags);
	
	/*** ground ***/
	CGSize size = [[CCDirector sharedDirector] winSize];
	
	// Define the ground body.
	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0, 0); // bottom-left corner
	b2Body *groundBody = world->CreateBody(&groundBodyDef);
	
	// Define the ground box shape.
	b2EdgeShape groundBox;
	
	// bottom
	groundBox.Set(b2Vec2(0,(size.height / 10.0)/PTM_RATIO), b2Vec2(size.width/PTM_RATIO,(size.height / 10.0)/PTM_RATIO));
	groundBody->CreateFixture(&groundBox,0);
	
	// top
	groundBox.Set(b2Vec2(0,size.height/PTM_RATIO), b2Vec2(size.width/PTM_RATIO,size.height/PTM_RATIO));
	groundBody->CreateFixture(&groundBox,0);
	
	// left
	groundBox.Set(b2Vec2(0,size.height/PTM_RATIO), b2Vec2(0,0));
	groundBody->CreateFixture(&groundBox,0);
	
	// right
	groundBox.Set(b2Vec2(size.width/PTM_RATIO,size.height/PTM_RATIO), b2Vec2(size.width/PTM_RATIO,0));
	groundBody->CreateFixture(&groundBox,0);
	
	/*** Static geometry ***/
    b2PolygonShape sd;
    sd.SetAsBox(5.0f, 0.5f);
    
    b2BodyDef bd;
    bd.type = b2_staticBody;
    bd.position.Set(12.5f, 7.0f);
    b2Body* ground = world->CreateBody(&bd);
	
    b2FixtureDef gFix;
    gFix.shape = &sd;
	
    ground->CreateFixture(&gFix);
    sd.SetAsBox(0.5f, 10.0f,b2Vec2(-7.5f,9.5f),0);
    ground->CreateFixture(&gFix);
    sd.SetAsBox(0.5f, 10.0f,b2Vec2(5.5f,9.5f),0);
    ground->CreateFixture(&gFix);
    sd.SetAsBox(4.5f, 0.5f,b2Vec2(0.0f,4.0f),0.57f);
    ground->CreateFixture(&gFix);
	
	b2CircleShape cd;
    cd.m_radius = 0.5f;
    cd.m_p = b2Vec2(-6.0f,-3.0f);
    gFix.shape = &cd;
    ground->CreateFixture(&gFix);
}

+(void)initializeScene
{
	CGSize size = [CCDirector sharedDirector].winSize;
	
	// set up sprite
	batch = [[CCSpriteBatchNode batchNodeWithFile:@"drop.png" capacity:1000] retain];
	
	// render texture
	renderTexture = [DLRenderTexture renderTextureWithWidth:size.width height:size.height pixelFormat:kCCTexture2DPixelFormat_RGBA4444];
	renderTexture.position = ccp(size.width / 2, size.height / 2);
	[currentScene addChild:renderTexture];
	
	// create menu
	[self createMenu];
	
	// override init for scene
	[currentScene initializeScene];
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
		batch = nil;
	}
	
	if (renderTexture)
	{
		[renderTexture release];
		renderTexture = nil;
	}	
}

+(void)createMenu
{
	// ask director for the window size
	CGSize size = [[CCDirector sharedDirector] winSize];
	
	/*** name of scene ***/
	// create and initialize a Label
	CCLabelTTF *name_scene = [CCLabelTTF labelWithString:[nameScene objectAtIndex:currentSceneIndex] fontName:@"Marker Felt" fontSize:32];
	// position the label on the center of the screen
	name_scene.anchorPoint = ccp(0.5f, 0);
	name_scene.position =  ccp(size.width /2 , size.height / 10 - name_scene.contentSize.height);
	// add the label as a child to this Layer
	[currentScene addChild: name_scene];
	
	/*** count of particles ***/
	// create and initialize a Label
	particlesCountLabel = [CCLabelTTF labelWithString:[NSString stringWithFormat:@"%d",particlesCount] fontName:@"Marker Felt" fontSize:16];
	// position the label on the center of the screen
	particlesCountLabel.anchorPoint = ccp(0, 0);
	particlesCountLabel.position =  ccp(0 , size.height / 10 - particlesCountLabel.contentSize.height);
	// add the label as a child to this Layer
	[currentScene addChild: particlesCountLabel];
	
	/*** switchers for scene ***/
	[CCMenuItemFont setFontSize:labelFontSize];
	// Menu Item using blocks
	CCMenuItem *previous_scene = [CCMenuItemFont itemFromString:@"Previous" block:^(id sender)
								  {
									  currentSceneIndex--;
									  if (currentSceneIndex < 0)
									  {
										  currentSceneIndex = 0;
									  }
									  else
									  {
										  [self createPreviousScene];
									  }
								  }];
	// Menu Item using blocks
	CCMenuItem *next_scene = [CCMenuItemFont itemFromString:@"Next" block:^(id sender)
							  {
								  currentSceneIndex++;
								  if (currentSceneIndex > maxSceneIndex)
								  {
									  currentSceneIndex = maxSceneIndex;
								  }
								  else
								  {
									  [self createNextScene];
								  }
							  }];

	
	CCMenu *menu = [CCMenu menuWithItems:previous_scene, next_scene, nil];
	[menu alignItemsHorizontallyWithPadding:20];
	[menu setPosition:ccp(size.width/2, name_scene.position.y - labelFontSize)];
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
	[menu setPosition:ccp(size.width - labelFontSize, name_scene.position.y - labelFontSize / 2)];
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
	
#if 1 == DEBUG
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
