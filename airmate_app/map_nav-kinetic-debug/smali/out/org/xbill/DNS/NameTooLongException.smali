.class public Lorg/xbill/DNS/NameTooLongException;
.super Lorg/xbill/DNS/WireParseException;
.source "NameTooLongException.java"


# direct methods
.method public constructor <init>()V
    .registers 1

    .line 16
    invoke-direct {p0}, Lorg/xbill/DNS/WireParseException;-><init>()V

    .line 17
    return-void
.end method

.method public constructor <init>(Ljava/lang/String;)V
    .registers 2
    .param p1, "s"    # Ljava/lang/String;

    .line 21
    invoke-direct {p0, p1}, Lorg/xbill/DNS/WireParseException;-><init>(Ljava/lang/String;)V

    .line 22
    return-void
.end method
