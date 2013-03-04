//
//  AppDelegate.h
//  ww
//
//  Created by Alexey Tyurin on 25.02.13.
//  Copyright __MyCompanyName__ 2013. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface AppDelegate : NSObject <UIApplicationDelegate> {
	UIWindow			*window;
	UIViewController	*viewController;
}

@property (nonatomic, retain) UIWindow *window;

@end
