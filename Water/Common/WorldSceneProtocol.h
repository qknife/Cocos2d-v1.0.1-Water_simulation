//
//  WorldSceneProtocol.h
//  Water
//
//  Created by Alexey Tyurin on 03.03.13.
//
//

#import <Foundation/Foundation.h>

@protocol WorldSceneProtocol <NSObject>

-(void)initializeScene;
-(void)particlesCountUp:(NSInteger)diff_;
-(void)particlesCountDown:(NSInteger)diff_;

@end
