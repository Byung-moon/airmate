.class final enum Lorg/ros/internal/node/service/ServiceResponseDecoderState;
.super Ljava/lang/Enum;
.source "ServiceResponseDecoderState.java"


# annotations
.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/lang/Enum<",
        "Lorg/ros/internal/node/service/ServiceResponseDecoderState;",
        ">;"
    }
.end annotation


# static fields
.field private static final synthetic $VALUES:[Lorg/ros/internal/node/service/ServiceResponseDecoderState;

.field public static final enum ERROR_CODE:Lorg/ros/internal/node/service/ServiceResponseDecoderState;

.field public static final enum MESSAGE:Lorg/ros/internal/node/service/ServiceResponseDecoderState;

.field public static final enum MESSAGE_LENGTH:Lorg/ros/internal/node/service/ServiceResponseDecoderState;


# direct methods
.method static constructor <clinit>()V
    .registers 5

    .line 4
    new-instance v0, Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    const-string v1, "ERROR_CODE"

    const/4 v2, 0x0

    invoke-direct {v0, v1, v2}, Lorg/ros/internal/node/service/ServiceResponseDecoderState;-><init>(Ljava/lang/String;I)V

    sput-object v0, Lorg/ros/internal/node/service/ServiceResponseDecoderState;->ERROR_CODE:Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    new-instance v0, Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    const-string v1, "MESSAGE_LENGTH"

    const/4 v3, 0x1

    invoke-direct {v0, v1, v3}, Lorg/ros/internal/node/service/ServiceResponseDecoderState;-><init>(Ljava/lang/String;I)V

    sput-object v0, Lorg/ros/internal/node/service/ServiceResponseDecoderState;->MESSAGE_LENGTH:Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    new-instance v0, Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    const-string v1, "MESSAGE"

    const/4 v4, 0x2

    invoke-direct {v0, v1, v4}, Lorg/ros/internal/node/service/ServiceResponseDecoderState;-><init>(Ljava/lang/String;I)V

    sput-object v0, Lorg/ros/internal/node/service/ServiceResponseDecoderState;->MESSAGE:Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    .line 3
    const/4 v0, 0x3

    new-array v0, v0, [Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    sget-object v1, Lorg/ros/internal/node/service/ServiceResponseDecoderState;->ERROR_CODE:Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    aput-object v1, v0, v2

    sget-object v1, Lorg/ros/internal/node/service/ServiceResponseDecoderState;->MESSAGE_LENGTH:Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    aput-object v1, v0, v3

    sget-object v1, Lorg/ros/internal/node/service/ServiceResponseDecoderState;->MESSAGE:Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    aput-object v1, v0, v4

    sput-object v0, Lorg/ros/internal/node/service/ServiceResponseDecoderState;->$VALUES:[Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    return-void
.end method

.method private constructor <init>(Ljava/lang/String;I)V
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()V"
        }
    .end annotation

    .line 3
    invoke-direct {p0, p1, p2}, Ljava/lang/Enum;-><init>(Ljava/lang/String;I)V

    return-void
.end method

.method public static valueOf(Ljava/lang/String;)Lorg/ros/internal/node/service/ServiceResponseDecoderState;
    .registers 2
    .param p0, "name"    # Ljava/lang/String;

    .line 3
    const-class v0, Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    invoke-static {v0, p0}, Ljava/lang/Enum;->valueOf(Ljava/lang/Class;Ljava/lang/String;)Ljava/lang/Enum;

    move-result-object v0

    check-cast v0, Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    return-object v0
.end method

.method public static values()[Lorg/ros/internal/node/service/ServiceResponseDecoderState;
    .registers 1

    .line 3
    sget-object v0, Lorg/ros/internal/node/service/ServiceResponseDecoderState;->$VALUES:[Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    invoke-virtual {v0}, [Lorg/ros/internal/node/service/ServiceResponseDecoderState;->clone()Ljava/lang/Object;

    move-result-object v0

    check-cast v0, [Lorg/ros/internal/node/service/ServiceResponseDecoderState;

    return-object v0
.end method
