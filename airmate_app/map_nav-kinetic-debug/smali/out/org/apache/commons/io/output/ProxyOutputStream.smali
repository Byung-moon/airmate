.class public Lorg/apache/commons/io/output/ProxyOutputStream;
.super Ljava/io/FilterOutputStream;
.source "ProxyOutputStream.java"


# direct methods
.method public constructor <init>(Ljava/io/OutputStream;)V
    .registers 2
    .param p1, "proxy"    # Ljava/io/OutputStream;

    .line 40
    invoke-direct {p0, p1}, Ljava/io/FilterOutputStream;-><init>(Ljava/io/OutputStream;)V

    .line 42
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
    iget-object v0, p0, Lorg/apache/commons/io/output/ProxyOutputStream;->out:Ljava/io/OutputStream;

    invoke-virtual {v0}, Ljava/io/OutputStream;->close()V

    .line 87
    return-void
.end method

.method public flush()V
    .registers 2
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 78
    iget-object v0, p0, Lorg/apache/commons/io/output/ProxyOutputStream;->out:Ljava/io/OutputStream;

    invoke-virtual {v0}, Ljava/io/OutputStream;->flush()V

    .line 79
    return-void
.end method

.method public write(I)V
    .registers 3
    .param p1, "idx"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 50
    iget-object v0, p0, Lorg/apache/commons/io/output/ProxyOutputStream;->out:Ljava/io/OutputStream;

    invoke-virtual {v0, p1}, Ljava/io/OutputStream;->write(I)V

    .line 51
    return-void
.end method

.method public write([B)V
    .registers 3
    .param p1, "bts"    # [B
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 59
    iget-object v0, p0, Lorg/apache/commons/io/output/ProxyOutputStream;->out:Ljava/io/OutputStream;

    invoke-virtual {v0, p1}, Ljava/io/OutputStream;->write([B)V

    .line 60
    return-void
.end method

.method public write([BII)V
    .registers 5
    .param p1, "bts"    # [B
    .param p2, "st"    # I
    .param p3, "end"    # I
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 70
    iget-object v0, p0, Lorg/apache/commons/io/output/ProxyOutputStream;->out:Ljava/io/OutputStream;

    invoke-virtual {v0, p1, p2, p3}, Ljava/io/OutputStream;->write([BII)V

    .line 71
    return-void
.end method
