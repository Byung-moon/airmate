.class public final Lorg/apache/commons/net/chargen/CharGenTCPClient;
.super Lorg/apache/commons/net/SocketClient;
.source "CharGenTCPClient.java"


# static fields
.field public static final CHARGEN_PORT:I = 0x13

.field public static final DEFAULT_PORT:I = 0x13

.field public static final NETSTAT_PORT:I = 0xf

.field public static final QUOTE_OF_DAY_PORT:I = 0x11

.field public static final SYSTAT_PORT:I = 0xb


# direct methods
.method public constructor <init>()V
    .registers 2

    .line 65
    invoke-direct {p0}, Lorg/apache/commons/net/SocketClient;-><init>()V

    .line 66
    const/16 v0, 0x13

    invoke-virtual {p0, v0}, Lorg/apache/commons/net/chargen/CharGenTCPClient;->setDefaultPort(I)V

    .line 67
    return-void
.end method


# virtual methods
.method public getInputStream()Ljava/io/InputStream;
    .registers 2

    .line 80
    iget-object v0, p0, Lorg/apache/commons/net/chargen/CharGenTCPClient;->_input_:Ljava/io/InputStream;

    return-object v0
.end method