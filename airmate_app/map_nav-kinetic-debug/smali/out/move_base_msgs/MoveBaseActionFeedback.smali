.class public interface abstract Lmove_base_msgs/MoveBaseActionFeedback;
.super Ljava/lang/Object;
.source "MoveBaseActionFeedback.java"

# interfaces
.implements Lorg/ros/internal/message/Message;


# static fields
.field public static final _DEFINITION:Ljava/lang/String; = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\nMoveBaseFeedback feedback\n"

.field public static final _TYPE:Ljava/lang/String; = "move_base_msgs/MoveBaseActionFeedback"


# virtual methods
.method public abstract getFeedback()Lmove_base_msgs/MoveBaseFeedback;
.end method

.method public abstract getHeader()Lstd_msgs/Header;
.end method

.method public abstract getStatus()Lactionlib_msgs/GoalStatus;
.end method

.method public abstract setFeedback(Lmove_base_msgs/MoveBaseFeedback;)V
.end method

.method public abstract setHeader(Lstd_msgs/Header;)V
.end method

.method public abstract setStatus(Lactionlib_msgs/GoalStatus;)V
.end method
