.class public interface abstract Lnav_msgs/GetMapActionResult;
.super Ljava/lang/Object;
.source "GetMapActionResult.java"

# interfaces
.implements Lorg/ros/internal/message/Message;


# static fields
.field public static final _DEFINITION:Ljava/lang/String; = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\nnav_msgs/GetMapResult result"

.field public static final _TYPE:Ljava/lang/String; = "nav_msgs/GetMapActionResult"


# virtual methods
.method public abstract getHeader()Lstd_msgs/Header;
.end method

.method public abstract getResult()Lnav_msgs/GetMapResult;
.end method

.method public abstract getStatus()Lactionlib_msgs/GoalStatus;
.end method

.method public abstract setHeader(Lstd_msgs/Header;)V
.end method

.method public abstract setResult(Lnav_msgs/GetMapResult;)V
.end method

.method public abstract setStatus(Lactionlib_msgs/GoalStatus;)V
.end method
