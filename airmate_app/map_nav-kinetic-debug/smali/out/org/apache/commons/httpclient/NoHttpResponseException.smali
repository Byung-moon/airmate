.class public Lorg/apache/commons/httpclient/NoHttpResponseException;
.super Ljava/io/IOException;
.source "NoHttpResponseException.java"


# direct methods
.method public constructor <init>()V
    .registers 1

    .line 52
    invoke-direct {p0}, Ljava/io/IOException;-><init>()V

    .line 53
    return-void
.end method

.method public constructor <init>(Ljava/lang/String;)V
    .registers 2
    .param p1, "message"    # Ljava/lang/String;

    .line 61
    invoke-direct {p0, p1}, Ljava/io/IOException;-><init>(Ljava/lang/String;)V

    .line 62
    return-void
.end method

.method public constructor <init>(Ljava/lang/String;Ljava/lang/Throwable;)V
    .registers 3
    .param p1, "message"    # Ljava/lang/String;
    .param p2, "cause"    # Ljava/lang/Throwable;

    .line 74
    invoke-direct {p0, p1}, Ljava/io/IOException;-><init>(Ljava/lang/String;)V

    .line 76
    invoke-static {p0, p2}, Lorg/apache/commons/httpclient/util/ExceptionUtil;->initCause(Ljava/lang/Throwable;Ljava/lang/Throwable;)V

    .line 77
    return-void
.end method