.class public interface abstract Lorg/jboss/netty/util/ThreadNameDeterminer;
.super Ljava/lang/Object;
.source "ThreadNameDeterminer.java"


# static fields
.field public static final CURRENT:Lorg/jboss/netty/util/ThreadNameDeterminer;

.field public static final PROPOSED:Lorg/jboss/netty/util/ThreadNameDeterminer;


# direct methods
.method static constructor <clinit>()V
    .registers 1

    .line 27
    new-instance v0, Lorg/jboss/netty/util/ThreadNameDeterminer$1;

    invoke-direct {v0}, Lorg/jboss/netty/util/ThreadNameDeterminer$1;-><init>()V

    sput-object v0, Lorg/jboss/netty/util/ThreadNameDeterminer;->PROPOSED:Lorg/jboss/netty/util/ThreadNameDeterminer;

    .line 38
    new-instance v0, Lorg/jboss/netty/util/ThreadNameDeterminer$2;

    invoke-direct {v0}, Lorg/jboss/netty/util/ThreadNameDeterminer$2;-><init>()V

    sput-object v0, Lorg/jboss/netty/util/ThreadNameDeterminer;->CURRENT:Lorg/jboss/netty/util/ThreadNameDeterminer;

    return-void
.end method


# virtual methods
.method public abstract determineThreadName(Ljava/lang/String;Ljava/lang/String;)Ljava/lang/String;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/lang/Exception;
        }
    .end annotation
.end method
