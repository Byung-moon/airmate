.class public interface abstract Lgeometry_msgs/Accel;
.super Ljava/lang/Object;
.source "Accel.java"

# interfaces
.implements Lorg/ros/internal/message/Message;


# static fields
.field public static final _DEFINITION:Ljava/lang/String; = "# This expresses acceleration in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n"

.field public static final _TYPE:Ljava/lang/String; = "geometry_msgs/Accel"


# virtual methods
.method public abstract getAngular()Lgeometry_msgs/Vector3;
.end method

.method public abstract getLinear()Lgeometry_msgs/Vector3;
.end method

.method public abstract setAngular(Lgeometry_msgs/Vector3;)V
.end method

.method public abstract setLinear(Lgeometry_msgs/Vector3;)V
.end method
