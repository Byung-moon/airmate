.class final enum Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;
.super Ljava/lang/Enum;
.source "JZlib.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lorg/jboss/netty/util/internal/jzlib/JZlib;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x4018
    name = "WrapperType"
.end annotation

.annotation system Ldalvik/annotation/Signature;
    value = {
        "Ljava/lang/Enum<",
        "Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;",
        ">;"
    }
.end annotation


# static fields
.field private static final synthetic $VALUES:[Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

.field public static final enum GZIP:Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

.field public static final enum NONE:Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

.field public static final enum ZLIB:Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

.field public static final enum ZLIB_OR_NONE:Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;


# direct methods
.method static constructor <clinit>()V
    .registers 6

    .line 106
    new-instance v0, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    const-string v1, "NONE"

    const/4 v2, 0x0

    invoke-direct {v0, v1, v2}, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;-><init>(Ljava/lang/String;I)V

    sput-object v0, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;->NONE:Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    new-instance v0, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    const-string v1, "ZLIB"

    const/4 v3, 0x1

    invoke-direct {v0, v1, v3}, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;-><init>(Ljava/lang/String;I)V

    sput-object v0, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;->ZLIB:Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    new-instance v0, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    const-string v1, "GZIP"

    const/4 v4, 0x2

    invoke-direct {v0, v1, v4}, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;-><init>(Ljava/lang/String;I)V

    sput-object v0, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;->GZIP:Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    new-instance v0, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    const-string v1, "ZLIB_OR_NONE"

    const/4 v5, 0x3

    invoke-direct {v0, v1, v5}, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;-><init>(Ljava/lang/String;I)V

    sput-object v0, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;->ZLIB_OR_NONE:Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    .line 105
    const/4 v0, 0x4

    new-array v0, v0, [Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    sget-object v1, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;->NONE:Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    aput-object v1, v0, v2

    sget-object v1, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;->ZLIB:Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    aput-object v1, v0, v3

    sget-object v1, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;->GZIP:Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    aput-object v1, v0, v4

    sget-object v1, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;->ZLIB_OR_NONE:Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    aput-object v1, v0, v5

    sput-object v0, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;->$VALUES:[Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    return-void
.end method

.method private constructor <init>(Ljava/lang/String;I)V
    .registers 3
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()V"
        }
    .end annotation

    .line 105
    invoke-direct {p0, p1, p2}, Ljava/lang/Enum;-><init>(Ljava/lang/String;I)V

    return-void
.end method

.method public static valueOf(Ljava/lang/String;)Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;
    .registers 2
    .param p0, "name"    # Ljava/lang/String;

    .line 105
    const-class v0, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    invoke-static {v0, p0}, Ljava/lang/Enum;->valueOf(Ljava/lang/Class;Ljava/lang/String;)Ljava/lang/Enum;

    move-result-object v0

    check-cast v0, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    return-object v0
.end method

.method public static values()[Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;
    .registers 1

    .line 105
    sget-object v0, Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;->$VALUES:[Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    invoke-virtual {v0}, [Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;->clone()Ljava/lang/Object;

    move-result-object v0

    check-cast v0, [Lorg/jboss/netty/util/internal/jzlib/JZlib$WrapperType;

    return-object v0
.end method
