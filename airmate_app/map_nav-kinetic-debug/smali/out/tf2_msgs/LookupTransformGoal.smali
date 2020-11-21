.class public interface abstract Ltf2_msgs/LookupTransformGoal;
.super Ljava/lang/Object;
.source "LookupTransformGoal.java"

# interfaces
.implements Lorg/ros/internal/message/Message;


# static fields
.field public static final _DEFINITION:Ljava/lang/String; = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n#goal definition#Simple API\nstring target_frame\nstring source_frame\ntime source_time\nduration timeout\n\n#Advanced API\ntime target_time\nstring fixed_frame\n\n#Whether or not to use the advanced API\nbool advanced\n\n"

.field public static final _TYPE:Ljava/lang/String; = "tf2_msgs/LookupTransformGoal"


# virtual methods
.method public abstract getAdvanced()Z
.end method

.method public abstract getFixedFrame()Ljava/lang/String;
.end method

.method public abstract getSourceFrame()Ljava/lang/String;
.end method

.method public abstract getSourceTime()Lorg/ros/message/Time;
.end method

.method public abstract getTargetFrame()Ljava/lang/String;
.end method

.method public abstract getTargetTime()Lorg/ros/message/Time;
.end method

.method public abstract getTimeout()Lorg/ros/message/Duration;
.end method

.method public abstract setAdvanced(Z)V
.end method

.method public abstract setFixedFrame(Ljava/lang/String;)V
.end method

.method public abstract setSourceFrame(Ljava/lang/String;)V
.end method

.method public abstract setSourceTime(Lorg/ros/message/Time;)V
.end method

.method public abstract setTargetFrame(Ljava/lang/String;)V
.end method

.method public abstract setTargetTime(Lorg/ros/message/Time;)V
.end method

.method public abstract setTimeout(Lorg/ros/message/Duration;)V
.end method
