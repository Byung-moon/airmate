.class public Lorg/jboss/netty/channel/DefaultChildChannelStateEvent;
.super Ljava/lang/Object;
.source "DefaultChildChannelStateEvent.java"

# interfaces
.implements Lorg/jboss/netty/channel/ChildChannelStateEvent;


# instance fields
.field private final childChannel:Lorg/jboss/netty/channel/Channel;

.field private final parentChannel:Lorg/jboss/netty/channel/Channel;


# direct methods
.method public constructor <init>(Lorg/jboss/netty/channel/Channel;Lorg/jboss/netty/channel/Channel;)V
    .registers 5
    .param p1, "parentChannel"    # Lorg/jboss/netty/channel/Channel;
    .param p2, "childChannel"    # Lorg/jboss/netty/channel/Channel;

    .line 31
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 32
    if-eqz p1, :cond_14

    .line 35
    if-eqz p2, :cond_c

    .line 38
    iput-object p1, p0, Lorg/jboss/netty/channel/DefaultChildChannelStateEvent;->parentChannel:Lorg/jboss/netty/channel/Channel;

    .line 39
    iput-object p2, p0, Lorg/jboss/netty/channel/DefaultChildChannelStateEvent;->childChannel:Lorg/jboss/netty/channel/Channel;

    .line 40
    return-void

    .line 36
    :cond_c
    new-instance v0, Ljava/lang/NullPointerException;

    const-string v1, "childChannel"

    invoke-direct {v0, v1}, Ljava/lang/NullPointerException;-><init>(Ljava/lang/String;)V

    throw v0

    .line 33
    :cond_14
    new-instance v0, Ljava/lang/NullPointerException;

    const-string v1, "parentChannel"

    invoke-direct {v0, v1}, Ljava/lang/NullPointerException;-><init>(Ljava/lang/String;)V

    throw v0
.end method


# virtual methods
.method public getChannel()Lorg/jboss/netty/channel/Channel;
    .registers 2

    .line 43
    iget-object v0, p0, Lorg/jboss/netty/channel/DefaultChildChannelStateEvent;->parentChannel:Lorg/jboss/netty/channel/Channel;

    return-object v0
.end method

.method public getChildChannel()Lorg/jboss/netty/channel/Channel;
    .registers 2

    .line 51
    iget-object v0, p0, Lorg/jboss/netty/channel/DefaultChildChannelStateEvent;->childChannel:Lorg/jboss/netty/channel/Channel;

    return-object v0
.end method

.method public getFuture()Lorg/jboss/netty/channel/ChannelFuture;
    .registers 2

    .line 47
    invoke-virtual {p0}, Lorg/jboss/netty/channel/DefaultChildChannelStateEvent;->getChannel()Lorg/jboss/netty/channel/Channel;

    move-result-object v0

    invoke-static {v0}, Lorg/jboss/netty/channel/Channels;->succeededFuture(Lorg/jboss/netty/channel/Channel;)Lorg/jboss/netty/channel/ChannelFuture;

    move-result-object v0

    return-object v0
.end method

.method public toString()Ljava/lang/String;
    .registers 4

    .line 56
    invoke-virtual {p0}, Lorg/jboss/netty/channel/DefaultChildChannelStateEvent;->getChannel()Lorg/jboss/netty/channel/Channel;

    move-result-object v0

    invoke-virtual {v0}, Ljava/lang/Object;->toString()Ljava/lang/String;

    move-result-object v0

    .line 57
    .local v0, "channelString":Ljava/lang/String;
    new-instance v1, Ljava/lang/StringBuilder;

    invoke-virtual {v0}, Ljava/lang/String;->length()I

    move-result v2

    add-int/lit8 v2, v2, 0x20

    invoke-direct {v1, v2}, Ljava/lang/StringBuilder;-><init>(I)V

    .line 58
    .local v1, "buf":Ljava/lang/StringBuilder;
    invoke-virtual {v1, v0}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    .line 59
    invoke-virtual {p0}, Lorg/jboss/netty/channel/DefaultChildChannelStateEvent;->getChildChannel()Lorg/jboss/netty/channel/Channel;

    move-result-object v2

    invoke-interface {v2}, Lorg/jboss/netty/channel/Channel;->isOpen()Z

    move-result v2

    if-eqz v2, :cond_23

    const-string v2, " CHILD_OPEN: "

    goto :goto_25

    :cond_23
    const-string v2, " CHILD_CLOSED: "

    :goto_25
    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    .line 60
    invoke-virtual {p0}, Lorg/jboss/netty/channel/DefaultChildChannelStateEvent;->getChildChannel()Lorg/jboss/netty/channel/Channel;

    move-result-object v2

    invoke-interface {v2}, Lorg/jboss/netty/channel/Channel;->getId()Ljava/lang/Integer;

    move-result-object v2

    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/Object;)Ljava/lang/StringBuilder;

    .line 61
    invoke-virtual {v1}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v2

    return-object v2
.end method
