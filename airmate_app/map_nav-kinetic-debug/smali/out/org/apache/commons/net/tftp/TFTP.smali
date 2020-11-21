.class public Lorg/apache/commons/net/tftp/TFTP;
.super Lorg/apache/commons/net/DatagramSocketClient;
.source "TFTP.java"


# static fields
.field public static final ASCII_MODE:I = 0x0

.field public static final BINARY_MODE:I = 0x1

.field public static final DEFAULT_PORT:I = 0x45

.field public static final DEFAULT_TIMEOUT:I = 0x1388

.field public static final IMAGE_MODE:I = 0x1

.field public static final NETASCII_MODE:I = 0x0

.field public static final OCTET_MODE:I = 0x1

.field static final PACKET_SIZE:I = 0x204


# instance fields
.field private __receiveBuffer:[B

.field private __receiveDatagram:Ljava/net/DatagramPacket;

.field private __sendDatagram:Ljava/net/DatagramPacket;

.field _sendBuffer:[B


# direct methods
.method public constructor <init>()V
    .registers 2

    .line 129
    invoke-direct {p0}, Lorg/apache/commons/net/DatagramSocketClient;-><init>()V

    .line 130
    const/16 v0, 0x1388

    invoke-virtual {p0, v0}, Lorg/apache/commons/net/tftp/TFTP;->setDefaultTimeout(I)V

    .line 131
    const/4 v0, 0x0

    iput-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveBuffer:[B

    .line 132
    iput-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveDatagram:Ljava/net/DatagramPacket;

    .line 133
    return-void
.end method

.method public static final getModeName(I)Ljava/lang/String;
    .registers 2
    .param p0, "mode"    # I

    .line 121
    sget-object v0, Lorg/apache/commons/net/tftp/TFTPRequestPacket;->_modeStrings:[Ljava/lang/String;

    aget-object v0, v0, p0

    return-object v0
.end method


# virtual methods
.method public final beginBufferedOps()V
    .registers 5

    .line 241
    const/16 v0, 0x204

    new-array v1, v0, [B

    iput-object v1, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveBuffer:[B

    .line 242
    new-instance v1, Ljava/net/DatagramPacket;

    iget-object v2, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveBuffer:[B

    iget-object v3, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveBuffer:[B

    array-length v3, v3

    invoke-direct {v1, v2, v3}, Ljava/net/DatagramPacket;-><init>([BI)V

    iput-object v1, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveDatagram:Ljava/net/DatagramPacket;

    .line 244
    new-array v0, v0, [B

    iput-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->_sendBuffer:[B

    .line 245
    new-instance v0, Ljava/net/DatagramPacket;

    iget-object v1, p0, Lorg/apache/commons/net/tftp/TFTP;->_sendBuffer:[B

    iget-object v2, p0, Lorg/apache/commons/net/tftp/TFTP;->_sendBuffer:[B

    array-length v2, v2

    invoke-direct {v0, v1, v2}, Ljava/net/DatagramPacket;-><init>([BI)V

    iput-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->__sendDatagram:Ljava/net/DatagramPacket;

    .line 247
    return-void
.end method

.method public final bufferedReceive()Lorg/apache/commons/net/tftp/TFTPPacket;
    .registers 3
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;,
            Ljava/io/InterruptedIOException;,
            Ljava/net/SocketException;,
            Lorg/apache/commons/net/tftp/TFTPPacketException;
        }
    .end annotation

    .line 200
    iget-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveDatagram:Ljava/net/DatagramPacket;

    iget-object v1, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveBuffer:[B

    invoke-virtual {v0, v1}, Ljava/net/DatagramPacket;->setData([B)V

    .line 201
    iget-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveDatagram:Ljava/net/DatagramPacket;

    iget-object v1, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveBuffer:[B

    array-length v1, v1

    invoke-virtual {v0, v1}, Ljava/net/DatagramPacket;->setLength(I)V

    .line 202
    iget-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->_socket_:Ljava/net/DatagramSocket;

    iget-object v1, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveDatagram:Ljava/net/DatagramPacket;

    invoke-virtual {v0, v1}, Ljava/net/DatagramSocket;->receive(Ljava/net/DatagramPacket;)V

    .line 204
    iget-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveDatagram:Ljava/net/DatagramPacket;

    invoke-static {v0}, Lorg/apache/commons/net/tftp/TFTPPacket;->newTFTPPacket(Ljava/net/DatagramPacket;)Lorg/apache/commons/net/tftp/TFTPPacket;

    move-result-object v0

    return-object v0
.end method

.method public final bufferedSend(Lorg/apache/commons/net/tftp/TFTPPacket;)V
    .registers 5
    .param p1, "packet"    # Lorg/apache/commons/net/tftp/TFTPPacket;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 227
    iget-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->_socket_:Ljava/net/DatagramSocket;

    iget-object v1, p0, Lorg/apache/commons/net/tftp/TFTP;->__sendDatagram:Ljava/net/DatagramPacket;

    iget-object v2, p0, Lorg/apache/commons/net/tftp/TFTP;->_sendBuffer:[B

    invoke-virtual {p1, v1, v2}, Lorg/apache/commons/net/tftp/TFTPPacket;->_newDatagram(Ljava/net/DatagramPacket;[B)Ljava/net/DatagramPacket;

    move-result-object v1

    invoke-virtual {v0, v1}, Ljava/net/DatagramSocket;->send(Ljava/net/DatagramPacket;)V

    .line 228
    return-void
.end method

.method public final discardPackets()V
    .registers 4
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 147
    new-instance v0, Ljava/net/DatagramPacket;

    const/16 v1, 0x204

    new-array v2, v1, [B

    invoke-direct {v0, v2, v1}, Ljava/net/DatagramPacket;-><init>([BI)V

    .line 149
    .local v0, "datagram":Ljava/net/DatagramPacket;
    invoke-virtual {p0}, Lorg/apache/commons/net/tftp/TFTP;->getSoTimeout()I

    move-result v1

    .line 150
    .local v1, "to":I
    const/4 v2, 0x1

    invoke-virtual {p0, v2}, Lorg/apache/commons/net/tftp/TFTP;->setSoTimeout(I)V

    .line 155
    :goto_11
    :try_start_11
    iget-object v2, p0, Lorg/apache/commons/net/tftp/TFTP;->_socket_:Ljava/net/DatagramSocket;

    invoke-virtual {v2, v0}, Ljava/net/DatagramSocket;->receive(Ljava/net/DatagramPacket;)V
    :try_end_16
    .catch Ljava/net/SocketException; {:try_start_11 .. :try_end_16} :catch_19
    .catch Ljava/io/InterruptedIOException; {:try_start_11 .. :try_end_16} :catch_17

    goto :goto_11

    .line 161
    :catch_17
    move-exception v2

    goto :goto_1b

    .line 157
    :catch_19
    move-exception v2

    .line 164
    nop

    .line 166
    :goto_1b
    invoke-virtual {p0, v1}, Lorg/apache/commons/net/tftp/TFTP;->setSoTimeout(I)V

    .line 167
    return-void
.end method

.method public final endBufferedOps()V
    .registers 2

    .line 254
    const/4 v0, 0x0

    iput-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveBuffer:[B

    .line 255
    iput-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->__receiveDatagram:Ljava/net/DatagramPacket;

    .line 256
    iput-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->_sendBuffer:[B

    .line 257
    iput-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->__sendDatagram:Ljava/net/DatagramPacket;

    .line 258
    return-void
.end method

.method public final receive()Lorg/apache/commons/net/tftp/TFTPPacket;
    .registers 4
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;,
            Ljava/io/InterruptedIOException;,
            Ljava/net/SocketException;,
            Lorg/apache/commons/net/tftp/TFTPPacketException;
        }
    .end annotation

    .line 293
    new-instance v0, Ljava/net/DatagramPacket;

    const/16 v1, 0x204

    new-array v2, v1, [B

    invoke-direct {v0, v2, v1}, Ljava/net/DatagramPacket;-><init>([BI)V

    .line 295
    .local v0, "packet":Ljava/net/DatagramPacket;
    iget-object v1, p0, Lorg/apache/commons/net/tftp/TFTP;->_socket_:Ljava/net/DatagramSocket;

    invoke-virtual {v1, v0}, Ljava/net/DatagramSocket;->receive(Ljava/net/DatagramPacket;)V

    .line 297
    invoke-static {v0}, Lorg/apache/commons/net/tftp/TFTPPacket;->newTFTPPacket(Ljava/net/DatagramPacket;)Lorg/apache/commons/net/tftp/TFTPPacket;

    move-result-object v1

    return-object v1
.end method

.method public final send(Lorg/apache/commons/net/tftp/TFTPPacket;)V
    .registers 4
    .param p1, "packet"    # Lorg/apache/commons/net/tftp/TFTPPacket;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 269
    iget-object v0, p0, Lorg/apache/commons/net/tftp/TFTP;->_socket_:Ljava/net/DatagramSocket;

    invoke-virtual {p1}, Lorg/apache/commons/net/tftp/TFTPPacket;->newDatagram()Ljava/net/DatagramPacket;

    move-result-object v1

    invoke-virtual {v0, v1}, Ljava/net/DatagramSocket;->send(Ljava/net/DatagramPacket;)V

    .line 270
    return-void
.end method
