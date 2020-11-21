.class public final Lorg/apache/commons/net/time/TimeUDPClient;
.super Lorg/apache/commons/net/DatagramSocketClient;
.source "TimeUDPClient.java"


# static fields
.field public static final DEFAULT_PORT:I = 0x25

.field public static final SECONDS_1900_TO_1970:J = 0x83aa7e80L


# instance fields
.field private __dummyData:[B

.field private __timeData:[B


# direct methods
.method public constructor <init>()V
    .registers 2

    .line 47
    invoke-direct {p0}, Lorg/apache/commons/net/DatagramSocketClient;-><init>()V

    .line 59
    const/4 v0, 0x1

    new-array v0, v0, [B

    iput-object v0, p0, Lorg/apache/commons/net/time/TimeUDPClient;->__dummyData:[B

    .line 60
    const/4 v0, 0x4

    new-array v0, v0, [B

    iput-object v0, p0, Lorg/apache/commons/net/time/TimeUDPClient;->__timeData:[B

    return-void
.end method


# virtual methods
.method public getDate(Ljava/net/InetAddress;)Ljava/util/Date;
    .registers 7
    .param p1, "host"    # Ljava/net/InetAddress;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 122
    new-instance v0, Ljava/util/Date;

    const/16 v1, 0x25

    invoke-virtual {p0, p1, v1}, Lorg/apache/commons/net/time/TimeUDPClient;->getTime(Ljava/net/InetAddress;I)J

    move-result-wide v1

    const-wide v3, 0x83aa7e80L

    sub-long/2addr v1, v3

    const-wide/16 v3, 0x3e8

    mul-long v1, v1, v3

    invoke-direct {v0, v1, v2}, Ljava/util/Date;-><init>(J)V

    return-object v0
.end method

.method public getDate(Ljava/net/InetAddress;I)Ljava/util/Date;
    .registers 8
    .param p1, "host"    # Ljava/net/InetAddress;
    .param p2, "port"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 115
    new-instance v0, Ljava/util/Date;

    invoke-virtual {p0, p1, p2}, Lorg/apache/commons/net/time/TimeUDPClient;->getTime(Ljava/net/InetAddress;I)J

    move-result-wide v1

    const-wide v3, 0x83aa7e80L

    sub-long/2addr v1, v3

    const-wide/16 v3, 0x3e8

    mul-long v1, v1, v3

    invoke-direct {v0, v1, v2}, Ljava/util/Date;-><init>(J)V

    return-object v0
.end method

.method public getTime(Ljava/net/InetAddress;)J
    .registers 4
    .param p1, "host"    # Ljava/net/InetAddress;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 99
    const/16 v0, 0x25

    invoke-virtual {p0, p1, v0}, Lorg/apache/commons/net/time/TimeUDPClient;->getTime(Ljava/net/InetAddress;I)J

    move-result-wide v0

    return-wide v0
.end method

.method public getTime(Ljava/net/InetAddress;I)J
    .registers 11
    .param p1, "host"    # Ljava/net/InetAddress;
    .param p2, "port"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 80
    new-instance v0, Ljava/net/DatagramPacket;

    iget-object v1, p0, Lorg/apache/commons/net/time/TimeUDPClient;->__dummyData:[B

    iget-object v2, p0, Lorg/apache/commons/net/time/TimeUDPClient;->__dummyData:[B

    array-length v2, v2

    invoke-direct {v0, v1, v2, p1, p2}, Ljava/net/DatagramPacket;-><init>([BILjava/net/InetAddress;I)V

    .line 82
    .local v0, "sendPacket":Ljava/net/DatagramPacket;
    new-instance v1, Ljava/net/DatagramPacket;

    iget-object v2, p0, Lorg/apache/commons/net/time/TimeUDPClient;->__timeData:[B

    iget-object v3, p0, Lorg/apache/commons/net/time/TimeUDPClient;->__timeData:[B

    array-length v3, v3

    invoke-direct {v1, v2, v3}, Ljava/net/DatagramPacket;-><init>([BI)V

    .line 84
    .local v1, "receivePacket":Ljava/net/DatagramPacket;
    iget-object v2, p0, Lorg/apache/commons/net/time/TimeUDPClient;->_socket_:Ljava/net/DatagramSocket;

    invoke-virtual {v2, v0}, Ljava/net/DatagramSocket;->send(Ljava/net/DatagramPacket;)V

    .line 85
    iget-object v2, p0, Lorg/apache/commons/net/time/TimeUDPClient;->_socket_:Ljava/net/DatagramSocket;

    invoke-virtual {v2, v1}, Ljava/net/DatagramSocket;->receive(Ljava/net/DatagramPacket;)V

    .line 87
    const-wide/16 v2, 0x0

    .line 88
    .local v2, "time":J
    iget-object v4, p0, Lorg/apache/commons/net/time/TimeUDPClient;->__timeData:[B

    const/4 v5, 0x0

    aget-byte v4, v4, v5

    and-int/lit16 v4, v4, 0xff

    shl-int/lit8 v4, v4, 0x18

    int-to-long v4, v4

    const-wide v6, 0xffffffffL

    and-long/2addr v4, v6

    or-long/2addr v2, v4

    .line 89
    iget-object v4, p0, Lorg/apache/commons/net/time/TimeUDPClient;->__timeData:[B

    const/4 v5, 0x1

    aget-byte v4, v4, v5

    and-int/lit16 v4, v4, 0xff

    shl-int/lit8 v4, v4, 0x10

    int-to-long v4, v4

    and-long/2addr v4, v6

    or-long/2addr v2, v4

    .line 90
    iget-object v4, p0, Lorg/apache/commons/net/time/TimeUDPClient;->__timeData:[B

    const/4 v5, 0x2

    aget-byte v4, v4, v5

    and-int/lit16 v4, v4, 0xff

    shl-int/lit8 v4, v4, 0x8

    int-to-long v4, v4

    and-long/2addr v4, v6

    or-long/2addr v2, v4

    .line 91
    iget-object v4, p0, Lorg/apache/commons/net/time/TimeUDPClient;->__timeData:[B

    const/4 v5, 0x3

    aget-byte v4, v4, v5

    and-int/lit16 v4, v4, 0xff

    int-to-long v4, v4

    and-long/2addr v4, v6

    or-long/2addr v2, v4

    .line 93
    return-wide v2
.end method
