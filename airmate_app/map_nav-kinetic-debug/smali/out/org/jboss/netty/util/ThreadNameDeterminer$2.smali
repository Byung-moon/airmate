.class final Lorg/jboss/netty/util/ThreadNameDeterminer$2;
.super Ljava/lang/Object;
.source "ThreadNameDeterminer.java"

# interfaces
.implements Lorg/jboss/netty/util/ThreadNameDeterminer;


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lorg/jboss/netty/util/ThreadNameDeterminer;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x8
    name = null
.end annotation


# direct methods
.method constructor <init>()V
    .registers 1

    .line 38
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method


# virtual methods
.method public determineThreadName(Ljava/lang/String;Ljava/lang/String;)Ljava/lang/String;
    .registers 4
    .param p1, "currentThreadName"    # Ljava/lang/String;
    .param p2, "proposedThreadName"    # Ljava/lang/String;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/Exception;
        }
    .end annotation

    .line 41
    const/4 v0, 0x0

    return-object v0
.end method
