.class Lcom/google/common/cache/CacheBuilderSpec$KeyStrengthParser;
.super Ljava/lang/Object;
.source "CacheBuilderSpec.java"

# interfaces
.implements Lcom/google/common/cache/CacheBuilderSpec$ValueParser;


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lcom/google/common/cache/CacheBuilderSpec;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x8
    name = "KeyStrengthParser"
.end annotation


# instance fields
.field private final strength:Lcom/google/common/cache/LocalCache$Strength;


# direct methods
.method public constructor <init>(Lcom/google/common/cache/LocalCache$Strength;)V
    .registers 2
    .param p1, "strength"    # Lcom/google/common/cache/LocalCache$Strength;

    .line 358
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 359
    iput-object p1, p0, Lcom/google/common/cache/CacheBuilderSpec$KeyStrengthParser;->strength:Lcom/google/common/cache/LocalCache$Strength;

    .line 360
    return-void
.end method


# virtual methods
.method public parse(Lcom/google/common/cache/CacheBuilderSpec;Ljava/lang/String;Ljava/lang/String;)V
    .registers 9
    .param p1, "spec"    # Lcom/google/common/cache/CacheBuilderSpec;
    .param p2, "key"    # Ljava/lang/String;
    .param p3, "value"    # Ljava/lang/String;
        .annotation runtime Ljavax/annotation/Nullable;
        .end annotation
    .end param

    .line 364
    const/4 v0, 0x0

    const/4 v1, 0x1

    if-nez p3, :cond_6

    const/4 v2, 0x1

    goto :goto_7

    :cond_6
    const/4 v2, 0x0

    :goto_7
    const-string v3, "key %s does not take values"

    new-array v4, v1, [Ljava/lang/Object;

    aput-object p2, v4, v0

    invoke-static {v2, v3, v4}, Lcom/google/common/base/Preconditions;->checkArgument(ZLjava/lang/String;[Ljava/lang/Object;)V

    .line 365
    iget-object v2, p1, Lcom/google/common/cache/CacheBuilderSpec;->keyStrength:Lcom/google/common/cache/LocalCache$Strength;

    if-nez v2, :cond_16

    const/4 v2, 0x1

    goto :goto_17

    :cond_16
    const/4 v2, 0x0

    :goto_17
    const-string v3, "%s was already set to %s"

    const/4 v4, 0x2

    new-array v4, v4, [Ljava/lang/Object;

    aput-object p2, v4, v0

    iget-object v0, p1, Lcom/google/common/cache/CacheBuilderSpec;->keyStrength:Lcom/google/common/cache/LocalCache$Strength;

    aput-object v0, v4, v1

    invoke-static {v2, v3, v4}, Lcom/google/common/base/Preconditions;->checkArgument(ZLjava/lang/String;[Ljava/lang/Object;)V

    .line 366
    iget-object v0, p0, Lcom/google/common/cache/CacheBuilderSpec$KeyStrengthParser;->strength:Lcom/google/common/cache/LocalCache$Strength;

    iput-object v0, p1, Lcom/google/common/cache/CacheBuilderSpec;->keyStrength:Lcom/google/common/cache/LocalCache$Strength;

    .line 367
    return-void
.end method
