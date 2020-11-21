.class public Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;
.super Ljava/lang/Object;
.source "SharedResourceMisuseDetector.java"


# static fields
.field private static final MAX_ACTIVE_INSTANCES:I = 0x100

.field private static final logger:Lorg/jboss/netty/logging/InternalLogger;


# instance fields
.field private final activeInstances:Ljava/util/concurrent/atomic/AtomicLong;

.field private final logged:Ljava/util/concurrent/atomic/AtomicBoolean;

.field private final type:Ljava/lang/Class;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/lang/Class<",
            "*>;"
        }
    .end annotation
.end field


# direct methods
.method static constructor <clinit>()V
    .registers 1

    .line 30
    const-class v0, Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;

    invoke-static {v0}, Lorg/jboss/netty/logging/InternalLoggerFactory;->getInstance(Ljava/lang/Class;)Lorg/jboss/netty/logging/InternalLogger;

    move-result-object v0

    sput-object v0, Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;->logger:Lorg/jboss/netty/logging/InternalLogger;

    return-void
.end method

.method public constructor <init>(Ljava/lang/Class;)V
    .registers 4
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "(",
            "Ljava/lang/Class<",
            "*>;)V"
        }
    .end annotation

    .line 37
    .local p1, "type":Ljava/lang/Class;, "Ljava/lang/Class<*>;"
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 34
    new-instance v0, Ljava/util/concurrent/atomic/AtomicLong;

    invoke-direct {v0}, Ljava/util/concurrent/atomic/AtomicLong;-><init>()V

    iput-object v0, p0, Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;->activeInstances:Ljava/util/concurrent/atomic/AtomicLong;

    .line 35
    new-instance v0, Ljava/util/concurrent/atomic/AtomicBoolean;

    invoke-direct {v0}, Ljava/util/concurrent/atomic/AtomicBoolean;-><init>()V

    iput-object v0, p0, Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;->logged:Ljava/util/concurrent/atomic/AtomicBoolean;

    .line 38
    if-eqz p1, :cond_16

    .line 41
    iput-object p1, p0, Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;->type:Ljava/lang/Class;

    .line 42
    return-void

    .line 39
    :cond_16
    new-instance v0, Ljava/lang/NullPointerException;

    const-string v1, "type"

    invoke-direct {v0, v1}, Ljava/lang/NullPointerException;-><init>(Ljava/lang/String;)V

    throw v0
.end method


# virtual methods
.method public decrease()V
    .registers 2

    .line 59
    iget-object v0, p0, Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;->activeInstances:Ljava/util/concurrent/atomic/AtomicLong;

    invoke-virtual {v0}, Ljava/util/concurrent/atomic/AtomicLong;->decrementAndGet()J

    .line 60
    return-void
.end method

.method public increase()V
    .registers 6

    .line 45
    iget-object v0, p0, Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;->activeInstances:Ljava/util/concurrent/atomic/AtomicLong;

    invoke-virtual {v0}, Ljava/util/concurrent/atomic/AtomicLong;->incrementAndGet()J

    move-result-wide v0

    const-wide/16 v2, 0x100

    cmp-long v4, v0, v2

    if-lez v4, :cond_52

    .line 46
    sget-object v0, Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;->logger:Lorg/jboss/netty/logging/InternalLogger;

    invoke-interface {v0}, Lorg/jboss/netty/logging/InternalLogger;->isWarnEnabled()Z

    move-result v0

    if-eqz v0, :cond_52

    .line 47
    iget-object v0, p0, Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;->logged:Ljava/util/concurrent/atomic/AtomicBoolean;

    const/4 v1, 0x0

    const/4 v2, 0x1

    invoke-virtual {v0, v1, v2}, Ljava/util/concurrent/atomic/AtomicBoolean;->compareAndSet(ZZ)Z

    move-result v0

    if-eqz v0, :cond_52

    .line 48
    sget-object v0, Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;->logger:Lorg/jboss/netty/logging/InternalLogger;

    new-instance v1, Ljava/lang/StringBuilder;

    invoke-direct {v1}, Ljava/lang/StringBuilder;-><init>()V

    const-string v2, "You are creating too many "

    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    iget-object v2, p0, Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;->type:Ljava/lang/Class;

    invoke-virtual {v2}, Ljava/lang/Class;->getSimpleName()Ljava/lang/String;

    move-result-object v2

    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v2, " instances.  "

    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    iget-object v2, p0, Lorg/jboss/netty/util/internal/SharedResourceMisuseDetector;->type:Ljava/lang/Class;

    invoke-virtual {v2}, Ljava/lang/Class;->getSimpleName()Ljava/lang/String;

    move-result-object v2

    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v2, " is a shared resource that must be reused across the"

    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    const-string v2, " application, so that only a few instances are created."

    invoke-virtual {v1, v2}, Ljava/lang/StringBuilder;->append(Ljava/lang/String;)Ljava/lang/StringBuilder;

    invoke-virtual {v1}, Ljava/lang/StringBuilder;->toString()Ljava/lang/String;

    move-result-object v1

    invoke-interface {v0, v1}, Lorg/jboss/netty/logging/InternalLogger;->warn(Ljava/lang/String;)V

    .line 56
    :cond_52
    return-void
.end method
