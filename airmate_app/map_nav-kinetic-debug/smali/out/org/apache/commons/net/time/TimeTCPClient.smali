.class public final Lorg/apache/commons/net/time/TimeTCPClient;
.super Lorg/apache/commons/net/SocketClient;
.source "TimeTCPClient.java"


# static fields
.field public static final DEFAULT_PORT:I = 0x25

.field public static final SECONDS_1900_TO_1970:J = 0x83aa7e80L


# direct methods
.method public constructor <init>()V
    .registers 2

    .line 58
    invoke-direct {p0}, Lorg/apache/commons/net/SocketClient;-><init>()V

    .line 59
    const/16 v0, 0x25

    invoke-virtual {p0, v0}, Lorg/apache/commons/net/time/TimeTCPClient;->setDefaultPort(I)V

    .line 60
    return-void
.end method


# virtual methods
.method public getDate()Ljava/util/Date;
    .registers 6
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 105
    new-instance v0, Ljava/util/Date;

    invoke-virtual {p0}, Lorg/apache/commons/net/time/TimeTCPClient;->getTime()J

    move-result-wide v1

    const-wide v3, 0x83aa7e80L

    sub-long/2addr v1, v3

    const-wide/16 v3, 0x3e8

    mul-long v1, v1, v3

    invoke-direct {v0, v1, v2}, Ljava/util/Date;-><init>(J)V

    return-object v0
.end method

.method public getTime()J
    .registers 6
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 83
    new-instance v0, Ljava/io/DataInputStream;

    iget-object v1, p0, Lorg/apache/commons/net/time/TimeTCPClient;->_input_:Ljava/io/InputStream;

    invoke-direct {v0, v1}, Ljava/io/DataInputStream;-><init>(Ljava/io/InputStream;)V

    .line 84
    .local v0, "input":Ljava/io/DataInputStream;
    invoke-virtual {v0}, Ljava/io/DataInputStream;->readInt()I

    move-result v1

    int-to-long v1, v1

    const-wide v3, 0xffffffffL

    and-long/2addr v1, v3

    return-wide v1
.end method