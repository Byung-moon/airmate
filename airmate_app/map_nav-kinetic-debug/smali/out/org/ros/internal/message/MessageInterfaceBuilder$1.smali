.class synthetic Lorg/ros/internal/message/MessageInterfaceBuilder$1;
.super Ljava/lang/Object;
.source "MessageInterfaceBuilder.java"


# annotations
.annotation system Ldalvik/annotation/EnclosingClass;
    value = Lorg/ros/internal/message/MessageInterfaceBuilder;
.end annotation

.annotation system Ldalvik/annotation/InnerClass;
    accessFlags = 0x1008
    name = null
.end annotation


# static fields
.field static final synthetic $SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I


# direct methods
.method static constructor <clinit>()V
    .registers 3

    .line 132
    invoke-static {}, Lorg/ros/internal/message/field/PrimitiveFieldType;->values()[Lorg/ros/internal/message/field/PrimitiveFieldType;

    move-result-object v0

    array-length v0, v0

    new-array v0, v0, [I

    sput-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    :try_start_9
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->BOOL:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/4 v2, 0x1

    aput v2, v0, v1
    :try_end_14
    .catch Ljava/lang/NoSuchFieldError; {:try_start_9 .. :try_end_14} :catch_15

    goto :goto_16

    :catch_15
    move-exception v0

    :goto_16
    :try_start_16
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->FLOAT32:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/4 v2, 0x2

    aput v2, v0, v1
    :try_end_21
    .catch Ljava/lang/NoSuchFieldError; {:try_start_16 .. :try_end_21} :catch_22

    goto :goto_23

    :catch_22
    move-exception v0

    :goto_23
    :try_start_23
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->STRING:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/4 v2, 0x3

    aput v2, v0, v1
    :try_end_2e
    .catch Ljava/lang/NoSuchFieldError; {:try_start_23 .. :try_end_2e} :catch_2f

    goto :goto_30

    :catch_2f
    move-exception v0

    :goto_30
    :try_start_30
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->BYTE:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/4 v2, 0x4

    aput v2, v0, v1
    :try_end_3b
    .catch Ljava/lang/NoSuchFieldError; {:try_start_30 .. :try_end_3b} :catch_3c

    goto :goto_3d

    :catch_3c
    move-exception v0

    :goto_3d
    :try_start_3d
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->CHAR:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/4 v2, 0x5

    aput v2, v0, v1
    :try_end_48
    .catch Ljava/lang/NoSuchFieldError; {:try_start_3d .. :try_end_48} :catch_49

    goto :goto_4a

    :catch_49
    move-exception v0

    :goto_4a
    :try_start_4a
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->INT8:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/4 v2, 0x6

    aput v2, v0, v1
    :try_end_55
    .catch Ljava/lang/NoSuchFieldError; {:try_start_4a .. :try_end_55} :catch_56

    goto :goto_57

    :catch_56
    move-exception v0

    :goto_57
    :try_start_57
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->UINT8:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/4 v2, 0x7

    aput v2, v0, v1
    :try_end_62
    .catch Ljava/lang/NoSuchFieldError; {:try_start_57 .. :try_end_62} :catch_63

    goto :goto_64

    :catch_63
    move-exception v0

    :goto_64
    :try_start_64
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->INT16:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/16 v2, 0x8

    aput v2, v0, v1
    :try_end_70
    .catch Ljava/lang/NoSuchFieldError; {:try_start_64 .. :try_end_70} :catch_71

    goto :goto_72

    :catch_71
    move-exception v0

    :goto_72
    :try_start_72
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->UINT16:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/16 v2, 0x9

    aput v2, v0, v1
    :try_end_7e
    .catch Ljava/lang/NoSuchFieldError; {:try_start_72 .. :try_end_7e} :catch_7f

    goto :goto_80

    :catch_7f
    move-exception v0

    :goto_80
    :try_start_80
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->INT32:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/16 v2, 0xa

    aput v2, v0, v1
    :try_end_8c
    .catch Ljava/lang/NoSuchFieldError; {:try_start_80 .. :try_end_8c} :catch_8d

    goto :goto_8e

    :catch_8d
    move-exception v0

    :goto_8e
    :try_start_8e
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->UINT32:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/16 v2, 0xb

    aput v2, v0, v1
    :try_end_9a
    .catch Ljava/lang/NoSuchFieldError; {:try_start_8e .. :try_end_9a} :catch_9b

    goto :goto_9c

    :catch_9b
    move-exception v0

    :goto_9c
    :try_start_9c
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->INT64:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/16 v2, 0xc

    aput v2, v0, v1
    :try_end_a8
    .catch Ljava/lang/NoSuchFieldError; {:try_start_9c .. :try_end_a8} :catch_a9

    goto :goto_aa

    :catch_a9
    move-exception v0

    :goto_aa
    :try_start_aa
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->UINT64:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/16 v2, 0xd

    aput v2, v0, v1
    :try_end_b6
    .catch Ljava/lang/NoSuchFieldError; {:try_start_aa .. :try_end_b6} :catch_b7

    goto :goto_b8

    :catch_b7
    move-exception v0

    :goto_b8
    :try_start_b8
    sget-object v0, Lorg/ros/internal/message/MessageInterfaceBuilder$1;->$SwitchMap$org$ros$internal$message$field$PrimitiveFieldType:[I

    sget-object v1, Lorg/ros/internal/message/field/PrimitiveFieldType;->FLOAT64:Lorg/ros/internal/message/field/PrimitiveFieldType;

    invoke-virtual {v1}, Lorg/ros/internal/message/field/PrimitiveFieldType;->ordinal()I

    move-result v1

    const/16 v2, 0xe

    aput v2, v0, v1
    :try_end_c4
    .catch Ljava/lang/NoSuchFieldError; {:try_start_b8 .. :try_end_c4} :catch_c5

    goto :goto_c6

    :catch_c5
    move-exception v0

    :goto_c6
    return-void
.end method
