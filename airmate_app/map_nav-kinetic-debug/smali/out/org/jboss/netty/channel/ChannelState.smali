.class public final enum Lorg/jboss/netty/channel/ChannelState;
.super Ljava/lang/Enum;
.source "ChannelState.java"


# annotations
.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/lang/Enum<",
        "Lorg/jboss/netty/channel/ChannelState;",
        ">;"
    }
.end annotation


# static fields
.field private static final synthetic $VALUES:[Lorg/jboss/netty/channel/ChannelState;

.field public static final enum BOUND:Lorg/jboss/netty/channel/ChannelState;

.field public static final enum CONNECTED:Lorg/jboss/netty/channel/ChannelState;

.field public static final enum INTEREST_OPS:Lorg/jboss/netty/channel/ChannelState;

.field public static final enum OPEN:Lorg/jboss/netty/channel/ChannelState;


# direct methods
.method static constructor <clinit>()V
    .registers 6

    .line 95
    new-instance v0, Lorg/jboss/netty/channel/ChannelState;

    const-string v1, "OPEN"

    const/4 v2, 0x0

    invoke-direct {v0, v1, v2}, Lorg/jboss/netty/channel/ChannelState;-><init>(Ljava/lang/String;I)V

    sput-object v0, Lorg/jboss/netty/channel/ChannelState;->OPEN:Lorg/jboss/netty/channel/ChannelState;

    .line 100
    new-instance v0, Lorg/jboss/netty/channel/ChannelState;

    const-string v1, "BOUND"

    const/4 v3, 0x1

    invoke-direct {v0, v1, v3}, Lorg/jboss/netty/channel/ChannelState;-><init>(Ljava/lang/String;I)V

    sput-object v0, Lorg/jboss/netty/channel/ChannelState;->BOUND:Lorg/jboss/netty/channel/ChannelState;

    .line 106
    new-instance v0, Lorg/jboss/netty/channel/ChannelState;

    const-string v1, "CONNECTED"

    const/4 v4, 0x2

    invoke-direct {v0, v1, v4}, Lorg/jboss/netty/channel/ChannelState;-><init>(Ljava/lang/String;I)V

    sput-object v0, Lorg/jboss/netty/channel/ChannelState;->CONNECTED:Lorg/jboss/netty/channel/ChannelState;

    .line 112
    new-instance v0, Lorg/jboss/netty/channel/ChannelState;

    const-string v1, "INTEREST_OPS"

    const/4 v5, 0x3

    invoke-direct {v0, v1, v5}, Lorg/jboss/netty/channel/ChannelState;-><init>(Ljava/lang/String;I)V

    sput-object v0, Lorg/jboss/netty/channel/ChannelState;->INTEREST_OPS:Lorg/jboss/netty/channel/ChannelState;

    .line 91
    const/4 v0, 0x4

    new-array v0, v0, [Lorg/jboss/netty/channel/ChannelState;

    sget-object v1, Lorg/jboss/netty/channel/ChannelState;->OPEN:Lorg/jboss/netty/channel/ChannelState;

    aput-object v1, v0, v2

    sget-object v1, Lorg/jboss/netty/channel/ChannelState;->BOUND:Lorg/jboss/netty/channel/ChannelState;

    aput-object v1, v0, v3

    sget-object v1, Lorg/jboss/netty/channel/ChannelState;->CONNECTED:Lorg/jboss/netty/channel/ChannelState;

    aput-object v1, v0, v4

    sget-object v1, Lorg/jboss/netty/channel/ChannelState;->INTEREST_OPS:Lorg/jboss/netty/channel/ChannelState;

    aput-object v1, v0, v5

    sput-object v0, Lorg/jboss/netty/channel/ChannelState;->$VALUES:[Lorg/jboss/netty/channel/ChannelState;

    return-void
.end method

.method private constructor <init>(Ljava/lang/String;I)V
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()V"
        }
    .end annotation

    .line 91
    invoke-direct {p0, p1, p2}, Ljava/lang/Enum;-><init>(Ljava/lang/String;I)V

    return-void
.end method

.method public static valueOf(Ljava/lang/String;)Lorg/jboss/netty/channel/ChannelState;
    .registers 2
    .param p0, "name"    # Ljava/lang/String;

    .line 91
    const-class v0, Lorg/jboss/netty/channel/ChannelState;

    invoke-static {v0, p0}, Ljava/lang/Enum;->valueOf(Ljava/lang/Class;Ljava/lang/String;)Ljava/lang/Enum;

    move-result-object v0

    check-cast v0, Lorg/jboss/netty/channel/ChannelState;

    return-object v0
.end method

.method public static values()[Lorg/jboss/netty/channel/ChannelState;
    .registers 1

    .line 91
    sget-object v0, Lorg/jboss/netty/channel/ChannelState;->$VALUES:[Lorg/jboss/netty/channel/ChannelState;

    invoke-virtual {v0}, [Lorg/jboss/netty/channel/ChannelState;->clone()Ljava/lang/Object;

    move-result-object v0

    check-cast v0, [Lorg/jboss/netty/channel/ChannelState;

    return-object v0
.end method
