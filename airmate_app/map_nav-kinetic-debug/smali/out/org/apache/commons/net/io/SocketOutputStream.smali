.class public Lorg/apache/commons/net/io/SocketOutputStream;
.super Ljava/io/FilterOutputStream;
.source "SocketOutputStream.java"


# instance fields
.field private __socket:Ljava/net/Socket;


# direct methods
.method public constructor <init>(Ljava/net/Socket;Ljava/io/OutputStream;)V
    .registers 3
    .param p1, "socket"    # Ljava/net/Socket;
    .param p2, "stream"    # Ljava/io/OutputStream;

    .line 52
    invoke-direct {p0, p2}, Ljava/io/FilterOutputStream;-><init>(Ljava/io/OutputStream;)V

    .line 53
    iput-object p1, p0, Lorg/apache/commons/net/io/SocketOutputStream;->__socket:Ljava/net/Socket;

    .line 54
    return-void
.end method


# virtual methods
.method public close()V
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 86
    invoke-super {p0}, Ljava/io/FilterOutputStream;->close()V

    .line 87
    iget-object v0, p0, Lorg/apache/commons/net/io/SocketOutputStream;->__socket:Ljava/net/Socket;

    invoke-virtual {v0}, Ljava/net/Socket;->close()V

    .line 88
    return-void
.end method

.method public write([BII)V
    .registers 5
    .param p1, "buffer"    # [B
    .param p2, "offset"    # I
    .param p3, "length"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 72
    iget-object v0, p0, Lorg/apache/commons/net/io/SocketOutputStream;->out:Ljava/io/OutputStream;

    invoke-virtual {v0, p1, p2, p3}, Ljava/io/OutputStream;->write([BII)V

    .line 73
    return-void
.end method
