/*
 *  SocketMessages.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// SocketMessages.h - defines for socket messages

#ifndef SocketMessages_h
#define SocketMessages_h

const int kSocketMaxMessageLength = 64; // includes terminating 0
const char * kSocketRequestTaskMessage = "SM_RequestTask";
const char * kSocketSendingTask = "SM_SendTask";
const char * kSocketSendingScore = "SM_Score";
const char * kSocketBusy = "SM_Busy";

#endif
