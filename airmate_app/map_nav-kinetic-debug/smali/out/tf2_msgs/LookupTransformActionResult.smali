.class public interface abstract Ltf2_msgs/LookupTransformActionResult;
.super Ljava/lang/Object;
.source "LookupTransformActionResult.java"

# interfaces
.implements Lorg/ros/internal/message/Message;


# static fields
.field public static final _DEFINITION:Ljava/lang/String; = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\ntf2_msgs/LookupTransformResult result"

.field public static final _TYPE:Ljava/lang/String; = "tf2_msgs/LookupTransformActionResult"


# virtual methods
.method public abstract getHeader()Lstd_msgs/Header;
.end method

.method public abstract getResult()Ltf2_msgs/LookupTransformResult;
.end method

.method public abstract getStatus()Lactionlib_msgs/GoalStatus;
.end method

.method public abstract setHeader(Lstd_msgs/Header;)V
.end method

.method public abstract setResult(Ltf2_msgs/LookupTransformResult;)V
.end method

.method public abstract setStatus(Lactionlib_msgs/GoalStatus;)V
.end method