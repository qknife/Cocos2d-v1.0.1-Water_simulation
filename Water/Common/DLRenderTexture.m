//
//  DLRenderTexture.m
//  ww
//
//  Created by Alexey Tyurin on 25.02.13.
//
//

#import "DLRenderTexture.h"

@implementation DLRenderTexture

-(void)draw{
    // use alpha test to give it hard edges
    glEnable(GL_ALPHA_TEST);
    glAlphaFunc(GL_GREATER, 0.7f);
}

@end
