.class public Lorg/apache/commons/httpclient/ProxyClient$ConnectResponse;
.super Ljava/lang/Object;
.source "ProxyClient.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lorg/apache/commons/httpclient/ProxyClient;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x9
    name = "ConnectResponse"
.end annotation


# instance fields
.field private connectMethod:Lorg/apache/commons/httpclient/ConnectMethod;

.field private socket:Ljava/net/Socket;


# direct methods
.method private constructor <init>()V
    .registers 1

    .line 230
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method

.method synthetic constructor <init>(Lorg/apache/commons/httpclient/ProxyClient$1;)V
    .registers 2
    .param p1, "x0"    # Lorg/apache/commons/httpclient/ProxyClient$1;

    .line 224
    invoke-direct {p0}, Lorg/apache/commons/httpclient/ProxyClient$ConnectResponse;-><init>()V

    return-void
.end method

.method static synthetic access$100(Lorg/apache/commons/httpclient/ProxyClient$ConnectResponse;Lorg/apache/commons/httpclient/ConnectMethod;)V
    .registers 2
    .param p0, "x0"    # Lorg/apache/commons/httpclient/ProxyClient$ConnectResponse;
    .param p1, "x1"    # Lorg/apache/commons/httpclient/ConnectMethod;

    .line 224
    invoke-direct {p0, p1}, Lorg/apache/commons/httpclient/ProxyClient$ConnectResponse;->setConnectMethod(Lorg/apache/commons/httpclient/ConnectMethod;)V

    return-void
.end method

.method static synthetic access$200(Lorg/apache/commons/httpclient/ProxyClient$ConnectResponse;Ljava/net/Socket;)V
    .registers 2
    .param p0, "x0"    # Lorg/apache/commons/httpclient/ProxyClient$ConnectResponse;
    .param p1, "x1"    # Ljava/net/Socket;

    .line 224
    invoke-direct {p0, p1}, Lorg/apache/commons/httpclient/ProxyClient$ConnectResponse;->setSocket(Ljava/net/Socket;)V

    return-void
.end method

.method private setConnectMethod(Lorg/apache/commons/httpclient/ConnectMethod;)V
    .registers 2
    .param p1, "connectMethod"    # Lorg/apache/commons/httpclient/ConnectMethod;

    .line 245
    iput-object p1, p0, Lorg/apache/commons/httpclient/ProxyClient$ConnectResponse;->connectMethod:Lorg/apache/commons/httpclient/ConnectMethod;

    .line 246
    return-void
.end method

.method private setSocket(Ljava/net/Socket;)V
    .registers 2
    .param p1, "socket"    # Ljava/net/Socket;

    .line 261
    iput-object p1, p0, Lorg/apache/commons/httpclient/ProxyClient$ConnectResponse;->socket:Ljava/net/Socket;

    .line 262
    return-void
.end method


# virtual methods
.method public getConnectMethod()Lorg/apache/commons/httpclient/ConnectMethod;
    .registers 2

    .line 239
    iget-object v0, p0, Lorg/apache/commons/httpclient/ProxyClient$ConnectResponse;->connectMethod:Lorg/apache/commons/httpclient/ConnectMethod;

    return-object v0
.end method

.method public getSocket()Ljava/net/Socket;
    .registers 2

    .line 255
    iget-object v0, p0, Lorg/apache/commons/httpclient/ProxyClient$ConnectResponse;->socket:Ljava/net/Socket;

    return-object v0
.end method
