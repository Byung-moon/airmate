.class public Lorg/apache/commons/httpclient/methods/GetMethod;
.super Lorg/apache/commons/httpclient/HttpMethodBase;
.source "GetMethod.java"


# static fields
.field private static final LOG:Lorg/apache/commons/logging/Log;

.field static synthetic class$org$apache$commons$httpclient$methods$GetMethod:Ljava/lang/Class;


# direct methods
.method static constructor <clinit>()V
    .registers 1

    .line 68
    sget-object v0, Lorg/apache/commons/httpclient/methods/GetMethod;->class$org$apache$commons$httpclient$methods$GetMethod:Ljava/lang/Class;

    if-nez v0, :cond_d

    const-string v0, "org.apache.commons.httpclient.methods.GetMethod"

    invoke-static {v0}, Lorg/apache/commons/httpclient/methods/GetMethod;->class$(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v0

    sput-object v0, Lorg/apache/commons/httpclient/methods/GetMethod;->class$org$apache$commons$httpclient$methods$GetMethod:Ljava/lang/Class;

    goto :goto_f

    :cond_d
    sget-object v0, Lorg/apache/commons/httpclient/methods/GetMethod;->class$org$apache$commons$httpclient$methods$GetMethod:Ljava/lang/Class;

    :goto_f
    invoke-static {v0}, Lorg/apache/commons/logging/LogFactory;->getLog(Ljava/lang/Class;)Lorg/apache/commons/logging/Log;

    move-result-object v0

    sput-object v0, Lorg/apache/commons/httpclient/methods/GetMethod;->LOG:Lorg/apache/commons/logging/Log;

    return-void
.end method

.method public constructor <init>()V
    .registers 2

    .line 77
    invoke-direct {p0}, Lorg/apache/commons/httpclient/HttpMethodBase;-><init>()V

    .line 78
    const/4 v0, 0x1

    invoke-virtual {p0, v0}, Lorg/apache/commons/httpclient/methods/GetMethod;->setFollowRedirects(Z)V

    .line 79
    return-void
.end method

.method public constructor <init>(Ljava/lang/String;)V
    .registers 4
    .param p1, "uri"    # Ljava/lang/String;

    .line 89
    invoke-direct {p0, p1}, Lorg/apache/commons/httpclient/HttpMethodBase;-><init>(Ljava/lang/String;)V

    .line 90
    sget-object v0, Lorg/apache/commons/httpclient/methods/GetMethod;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter GetMethod(String)"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 91
    const/4 v0, 0x1

    invoke-virtual {p0, v0}, Lorg/apache/commons/httpclient/methods/GetMethod;->setFollowRedirects(Z)V

    .line 92
    return-void
.end method

.method static synthetic class$(Ljava/lang/String;)Ljava/lang/Class;
    .registers 4
    .param p0, "x0"    # Ljava/lang/String;

    .line 68
    :try_start_0
    invoke-static {p0}, Ljava/lang/Class;->forName(Ljava/lang/String;)Ljava/lang/Class;

    move-result-object v0
    :try_end_4
    .catch Ljava/lang/ClassNotFoundException; {:try_start_0 .. :try_end_4} :catch_5

    return-object v0

    :catch_5
    move-exception v0

    .local v0, "x1":Ljava/lang/ClassNotFoundException;
    new-instance v1, Ljava/lang/NoClassDefFoundError;

    invoke-virtual {v0}, Ljava/lang/ClassNotFoundException;->getMessage()Ljava/lang/String;

    move-result-object v2

    invoke-direct {v1, v2}, Ljava/lang/NoClassDefFoundError;-><init>(Ljava/lang/String;)V

    throw v1
.end method


# virtual methods
.method public getName()Ljava/lang/String;
    .registers 2

    .line 104
    const-string v0, "GET"

    return-object v0
.end method

.method public recycle()V
    .registers 3

    .line 123
    sget-object v0, Lorg/apache/commons/httpclient/methods/GetMethod;->LOG:Lorg/apache/commons/logging/Log;

    const-string v1, "enter GetMethod.recycle()"

    invoke-interface {v0, v1}, Lorg/apache/commons/logging/Log;->trace(Ljava/lang/Object;)V

    .line 125
    invoke-super {p0}, Lorg/apache/commons/httpclient/HttpMethodBase;->recycle()V

    .line 126
    const/4 v0, 0x1

    invoke-virtual {p0, v0}, Lorg/apache/commons/httpclient/methods/GetMethod;->setFollowRedirects(Z)V

    .line 127
    return-void
.end method
