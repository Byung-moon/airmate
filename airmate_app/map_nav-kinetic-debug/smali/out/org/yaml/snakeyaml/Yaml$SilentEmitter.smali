.class Lorg/yaml/snakeyaml/Yaml$SilentEmitter;
.super Ljava/lang/Object;
.source "Yaml.java"

# interfaces
.implements Lorg/yaml/snakeyaml/emitter/Emitable;


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lorg/yaml/snakeyaml/Yaml;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0xa
    name = "SilentEmitter"
.end annotation


# instance fields
.field private events:Ljava/util/List;
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "Ljava/util/List<",
            "Lorg/yaml/snakeyaml/events/Event;",
            ">;"
        }
    .end annotation
.end field


# direct methods
.method private constructor <init>()V
    .registers 3

    .line 378
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    .line 379
    new-instance v0, Ljava/util/ArrayList;

    const/16 v1, 0x64

    invoke-direct {v0, v1}, Ljava/util/ArrayList;-><init>(I)V

    iput-object v0, p0, Lorg/yaml/snakeyaml/Yaml$SilentEmitter;->events:Ljava/util/List;

    return-void
.end method

.method synthetic constructor <init>(Lorg/yaml/snakeyaml/Yaml$1;)V
    .registers 2
    .param p1, "x0"    # Lorg/yaml/snakeyaml/Yaml$1;

    .line 378
    invoke-direct {p0}, Lorg/yaml/snakeyaml/Yaml$SilentEmitter;-><init>()V

    return-void
.end method


# virtual methods
.method public emit(Lorg/yaml/snakeyaml/events/Event;)V
    .registers 3
    .param p1, "event"    # Lorg/yaml/snakeyaml/events/Event;
    .annotation system Ldalvik/annotation/Throws;
        value = {
            Ljava/io/IOException;
        }
    .end annotation

    .line 386
    iget-object v0, p0, Lorg/yaml/snakeyaml/Yaml$SilentEmitter;->events:Ljava/util/List;

    invoke-interface {v0, p1}, Ljava/util/List;->add(Ljava/lang/Object;)Z

    .line 387
    return-void
.end method

.method public getEvents()Ljava/util/List;
    .registers 2
    .annotation system Ldalvik/annotation/Signature;
        value = {
            "()",
            "Ljava/util/List<",
            "Lorg/yaml/snakeyaml/events/Event;",
            ">;"
        }
    .end annotation

    .line 382
    iget-object v0, p0, Lorg/yaml/snakeyaml/Yaml$SilentEmitter;->events:Ljava/util/List;

    return-object v0
.end method
