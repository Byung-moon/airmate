.class public final Lorg/apache/commons/net/tftp/TFTPWriteRequestPacket;
.super Lorg/apache/commons/net/tftp/TFTPRequestPacket;
.source "TFTPWriteRequestPacket.java"


# direct methods
.method constructor <init>(Ljava/net/DatagramPacket;)V
    .registers 3
    .param p1, "datagram"    # Ljava/net/DatagramPacket;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Lorg/apache/commons/net/tftp/TFTPPacketException;
        }
    .end annotation

    .line 77
    const/4 v0, 0x2

    invoke-direct {p0, v0, p1}, Lorg/apache/commons/net/tftp/TFTPRequestPacket;-><init>(ILjava/net/DatagramPacket;)V

    .line 78
    return-void
.end method

.method public constructor <init>(Ljava/net/InetAddress;ILjava/lang/String;I)V
    .registers 11
    .param p1, "destination"    # Ljava/net/InetAddress;
    .param p2, "port"    # I
    .param p3, "filename"    # Ljava/lang/String;
    .param p4, "mode"    # I

    .line 62
    const/4 v3, 0x2

    move-object v0, p0

    move-object v1, p1

    move v2, p2

    move-object v4, p3

    move v5, p4

    invoke-direct/range {v0 .. v5}, Lorg/apache/commons/net/tftp/TFTPRequestPacket;-><init>(Ljava/net/InetAddress;IILjava/lang/String;I)V

    .line 63
    return-void
.end method
