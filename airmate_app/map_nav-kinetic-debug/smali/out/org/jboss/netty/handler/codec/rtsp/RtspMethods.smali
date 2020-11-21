.class public final Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;
.super Ljava/lang/Object;
.source "RtspMethods.java"


# static fields
.field public static final ANNOUNCE:Lorg/jboss/netty/handler/codec/http/HttpMethod;

.field public static final DESCRIBE:Lorg/jboss/netty/handler/codec/http/HttpMethod;

.field public static final GET_PARAMETER:Lorg/jboss/netty/handler/codec/http/HttpMethod;

.field public static final OPTIONS:Lorg/jboss/netty/handler/codec/http/HttpMethod;

.field public static final PAUSE:Lorg/jboss/netty/handler/codec/http/HttpMethod;

.field public static final PLAY:Lorg/jboss/netty/handler/codec/http/HttpMethod;

.field public static final RECORD:Lorg/jboss/netty/handler/codec/http/HttpMethod;

.field public static final REDIRECT:Lorg/jboss/netty/handler/codec/http/HttpMethod;

.field public static final SETUP:Lorg/jboss/netty/handler/codec/http/HttpMethod;

.field public static final SET_PARAMETER:Lorg/jboss/netty/handler/codec/http/HttpMethod;

.field public static final TEARDOWN:Lorg/jboss/netty/handler/codec/http/HttpMethod;

.field private static final methodMap:Ljava/util/Map;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/Map<",
            "Ljava/lang/String;",
            "Lorg/jboss/netty/handler/codec/http/HttpMethod;",
            ">;"
        }
    .end annotation
.end field


# direct methods
.method static constructor <clinit>()V
    .registers 3

    .line 36
    sget-object v0, Lorg/jboss/netty/handler/codec/http/HttpMethod;->OPTIONS:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    sput-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->OPTIONS:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    .line 42
    new-instance v0, Lorg/jboss/netty/handler/codec/http/HttpMethod;

    const-string v1, "DESCRIBE"

    invoke-direct {v0, v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;-><init>(Ljava/lang/String;)V

    sput-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->DESCRIBE:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    .line 49
    new-instance v0, Lorg/jboss/netty/handler/codec/http/HttpMethod;

    const-string v1, "ANNOUNCE"

    invoke-direct {v0, v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;-><init>(Ljava/lang/String;)V

    sput-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->ANNOUNCE:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    .line 55
    new-instance v0, Lorg/jboss/netty/handler/codec/http/HttpMethod;

    const-string v1, "SETUP"

    invoke-direct {v0, v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;-><init>(Ljava/lang/String;)V

    sput-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->SETUP:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    .line 61
    new-instance v0, Lorg/jboss/netty/handler/codec/http/HttpMethod;

    const-string v1, "PLAY"

    invoke-direct {v0, v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;-><init>(Ljava/lang/String;)V

    sput-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->PLAY:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    .line 67
    new-instance v0, Lorg/jboss/netty/handler/codec/http/HttpMethod;

    const-string v1, "PAUSE"

    invoke-direct {v0, v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;-><init>(Ljava/lang/String;)V

    sput-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->PAUSE:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    .line 73
    new-instance v0, Lorg/jboss/netty/handler/codec/http/HttpMethod;

    const-string v1, "TEARDOWN"

    invoke-direct {v0, v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;-><init>(Ljava/lang/String;)V

    sput-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->TEARDOWN:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    .line 79
    new-instance v0, Lorg/jboss/netty/handler/codec/http/HttpMethod;

    const-string v1, "GET_PARAMETER"

    invoke-direct {v0, v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;-><init>(Ljava/lang/String;)V

    sput-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->GET_PARAMETER:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    .line 85
    new-instance v0, Lorg/jboss/netty/handler/codec/http/HttpMethod;

    const-string v1, "SET_PARAMETER"

    invoke-direct {v0, v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;-><init>(Ljava/lang/String;)V

    sput-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->SET_PARAMETER:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    .line 91
    new-instance v0, Lorg/jboss/netty/handler/codec/http/HttpMethod;

    const-string v1, "REDIRECT"

    invoke-direct {v0, v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;-><init>(Ljava/lang/String;)V

    sput-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->REDIRECT:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    .line 97
    new-instance v0, Lorg/jboss/netty/handler/codec/http/HttpMethod;

    const-string v1, "RECORD"

    invoke-direct {v0, v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;-><init>(Ljava/lang/String;)V

    sput-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->RECORD:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    .line 99
    new-instance v0, Ljava/util/HashMap;

    invoke-direct {v0}, Ljava/util/HashMap;-><init>()V

    sput-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    .line 102
    sget-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    sget-object v1, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->DESCRIBE:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;->toString()Ljava/lang/String;

    move-result-object v1

    sget-object v2, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->DESCRIBE:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-interface {v0, v1, v2}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 103
    sget-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    sget-object v1, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->ANNOUNCE:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;->toString()Ljava/lang/String;

    move-result-object v1

    sget-object v2, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->ANNOUNCE:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-interface {v0, v1, v2}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 104
    sget-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    sget-object v1, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->GET_PARAMETER:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;->toString()Ljava/lang/String;

    move-result-object v1

    sget-object v2, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->GET_PARAMETER:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-interface {v0, v1, v2}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 105
    sget-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    sget-object v1, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->OPTIONS:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;->toString()Ljava/lang/String;

    move-result-object v1

    sget-object v2, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->OPTIONS:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-interface {v0, v1, v2}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 106
    sget-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    sget-object v1, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->PAUSE:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;->toString()Ljava/lang/String;

    move-result-object v1

    sget-object v2, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->PAUSE:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-interface {v0, v1, v2}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 107
    sget-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    sget-object v1, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->PLAY:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;->toString()Ljava/lang/String;

    move-result-object v1

    sget-object v2, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->PLAY:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-interface {v0, v1, v2}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 108
    sget-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    sget-object v1, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->RECORD:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;->toString()Ljava/lang/String;

    move-result-object v1

    sget-object v2, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->RECORD:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-interface {v0, v1, v2}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 109
    sget-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    sget-object v1, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->REDIRECT:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;->toString()Ljava/lang/String;

    move-result-object v1

    sget-object v2, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->REDIRECT:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-interface {v0, v1, v2}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 110
    sget-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    sget-object v1, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->SETUP:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;->toString()Ljava/lang/String;

    move-result-object v1

    sget-object v2, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->SETUP:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-interface {v0, v1, v2}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 111
    sget-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    sget-object v1, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->SET_PARAMETER:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;->toString()Ljava/lang/String;

    move-result-object v1

    sget-object v2, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->SET_PARAMETER:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-interface {v0, v1, v2}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 112
    sget-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    sget-object v1, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->TEARDOWN:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-virtual {v1}, Lorg/jboss/netty/handler/codec/http/HttpMethod;->toString()Ljava/lang/String;

    move-result-object v1

    sget-object v2, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->TEARDOWN:Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-interface {v0, v1, v2}, Ljava/util/Map;->put(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;

    .line 113
    return-void
.end method

.method private constructor <init>()V
    .registers 1

    .line 139
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 140
    return-void
.end method

.method public static valueOf(Ljava/lang/String;)Lorg/jboss/netty/handler/codec/http/HttpMethod;
    .registers 3
    .param p0, "name"    # Ljava/lang/String;

    .line 121
    if-eqz p0, :cond_29

    .line 125
    invoke-virtual {p0}, Ljava/lang/String;->trim()Ljava/lang/String;

    move-result-object v0

    invoke-virtual {v0}, Ljava/lang/String;->toUpperCase()Ljava/lang/String;

    move-result-object p0

    .line 126
    invoke-virtual {p0}, Ljava/lang/String;->length()I

    move-result v0

    if-eqz v0, :cond_21

    .line 130
    sget-object v0, Lorg/jboss/netty/handler/codec/rtsp/RtspMethods;->methodMap:Ljava/util/Map;

    invoke-interface {v0, p0}, Ljava/util/Map;->get(Ljava/lang/Object;)Ljava/lang/Object;

    move-result-object v0

    check-cast v0, Lorg/jboss/netty/handler/codec/http/HttpMethod;

    .line 131
    .local v0, "result":Lorg/jboss/netty/handler/codec/http/HttpMethod;
    if-eqz v0, :cond_1b

    .line 132
    return-object v0

    .line 134
    :cond_1b
    new-instance v1, Lorg/jboss/netty/handler/codec/http/HttpMethod;

    invoke-direct {v1, p0}, Lorg/jboss/netty/handler/codec/http/HttpMethod;-><init>(Ljava/lang/String;)V

    return-object v1

    .line 127
    .end local v0    # "result":Lorg/jboss/netty/handler/codec/http/HttpMethod;
    :cond_21
    new-instance v0, Ljava/lang/IllegalArgumentException;

    const-string v1, "empty name"

    invoke-direct {v0, v1}, Ljava/lang/IllegalArgumentException;-><init>(Ljava/lang/String;)V

    throw v0

    .line 122
    :cond_29
    new-instance v0, Ljava/lang/NullPointerException;

    const-string v1, "name"

    invoke-direct {v0, v1}, Ljava/lang/NullPointerException;-><init>(Ljava/lang/String;)V

    throw v0
.end method
