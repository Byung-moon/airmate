.class public Lorg/apache/commons/httpclient/ConnectionPoolTimeoutException;
.super Lorg/apache/commons/httpclient/ConnectTimeoutException;
.source "ConnectionPoolTimeoutException.java"


# direct methods
.method public constructor <init>()V
    .registers 1

    .line 47
    invoke-direct {p0}, Lorg/apache/commons/httpclient/ConnectTimeoutException;-><init>()V

    .line 48
    return-void
.end method

.method public constructor <init>(Ljava/lang/String;)V
    .registers 2
    .param p1, "message"    # Ljava/lang/String;

    .line 56
    invoke-direct {p0, p1}, Lorg/apache/commons/httpclient/ConnectTimeoutException;-><init>(Ljava/lang/String;)V

    .line 57
    return-void
.end method

.method public constructor <init>(Ljava/lang/String;Ljava/lang/Throwable;)V
    .registers 3
    .param p1, "message"    # Ljava/lang/String;
    .param p2, "cause"    # Ljava/lang/Throwable;

    .line 67
    invoke-direct {p0, p1, p2}, Lorg/apache/commons/httpclient/ConnectTimeoutException;-><init>(Ljava/lang/String;Ljava/lang/Throwable;)V

    .line 68
    return-void
.end method