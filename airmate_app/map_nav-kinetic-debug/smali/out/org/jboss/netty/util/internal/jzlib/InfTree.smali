.class final Lorg/jboss/netty/util/internal/jzlib/InfTree;
.super Ljava/lang/Object;
.source "InfTree.java"


# static fields
.field static final BMAX:I = 0xf

.field static final cpdext:[I

.field static final cpdist:[I

.field static final cplens:[I

.field static final cplext:[I

.field static final fixed_bd:I = 0x5

.field static final fixed_bl:I = 0x9

.field static final fixed_td:[I

.field static final fixed_tl:[I


# instance fields
.field private c:[I

.field private hn:[I

.field private r:[I

.field private u:[I

.field private v:[I

.field private x:[I


# direct methods
.method static constructor <clinit>()V
    .registers 2

    .line 55
    const/16 v0, 0x600

    new-array v0, v0, [I

    fill-array-data v0, :array_34

    sput-object v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->fixed_tl:[I

    .line 139
    const/16 v0, 0x60

    new-array v0, v0, [I

    fill-array-data v0, :array_c38

    sput-object v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->fixed_td:[I

    .line 148
    const/16 v0, 0x1f

    new-array v1, v0, [I

    fill-array-data v1, :array_cfc

    sput-object v1, Lorg/jboss/netty/util/internal/jzlib/InfTree;->cplens:[I

    .line 153
    new-array v0, v0, [I

    fill-array-data v0, :array_d3e

    sput-object v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->cplext:[I

    .line 158
    const/16 v0, 0x1e

    new-array v1, v0, [I

    fill-array-data v1, :array_d80

    sput-object v1, Lorg/jboss/netty/util/internal/jzlib/InfTree;->cpdist:[I

    .line 162
    new-array v0, v0, [I

    fill-array-data v0, :array_dc0

    sput-object v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->cpdext:[I

    return-void

    nop

    :array_34
    .array-data 4
        0x60
        0x7
        0x100
        0x0
        0x8
        0x50
        0x0
        0x8
        0x10
        0x54
        0x8
        0x73
        0x52
        0x7
        0x1f
        0x0
        0x8
        0x70
        0x0
        0x8
        0x30
        0x0
        0x9
        0xc0
        0x50
        0x7
        0xa
        0x0
        0x8
        0x60
        0x0
        0x8
        0x20
        0x0
        0x9
        0xa0
        0x0
        0x8
        0x0
        0x0
        0x8
        0x80
        0x0
        0x8
        0x40
        0x0
        0x9
        0xe0
        0x50
        0x7
        0x6
        0x0
        0x8
        0x58
        0x0
        0x8
        0x18
        0x0
        0x9
        0x90
        0x53
        0x7
        0x3b
        0x0
        0x8
        0x78
        0x0
        0x8
        0x38
        0x0
        0x9
        0xd0
        0x51
        0x7
        0x11
        0x0
        0x8
        0x68
        0x0
        0x8
        0x28
        0x0
        0x9
        0xb0
        0x0
        0x8
        0x8
        0x0
        0x8
        0x88
        0x0
        0x8
        0x48
        0x0
        0x9
        0xf0
        0x50
        0x7
        0x4
        0x0
        0x8
        0x54
        0x0
        0x8
        0x14
        0x55
        0x8
        0xe3
        0x53
        0x7
        0x2b
        0x0
        0x8
        0x74
        0x0
        0x8
        0x34
        0x0
        0x9
        0xc8
        0x51
        0x7
        0xd
        0x0
        0x8
        0x64
        0x0
        0x8
        0x24
        0x0
        0x9
        0xa8
        0x0
        0x8
        0x4
        0x0
        0x8
        0x84
        0x0
        0x8
        0x44
        0x0
        0x9
        0xe8
        0x50
        0x7
        0x8
        0x0
        0x8
        0x5c
        0x0
        0x8
        0x1c
        0x0
        0x9
        0x98
        0x54
        0x7
        0x53
        0x0
        0x8
        0x7c
        0x0
        0x8
        0x3c
        0x0
        0x9
        0xd8
        0x52
        0x7
        0x17
        0x0
        0x8
        0x6c
        0x0
        0x8
        0x2c
        0x0
        0x9
        0xb8
        0x0
        0x8
        0xc
        0x0
        0x8
        0x8c
        0x0
        0x8
        0x4c
        0x0
        0x9
        0xf8
        0x50
        0x7
        0x3
        0x0
        0x8
        0x52
        0x0
        0x8
        0x12
        0x55
        0x8
        0xa3
        0x53
        0x7
        0x23
        0x0
        0x8
        0x72
        0x0
        0x8
        0x32
        0x0
        0x9
        0xc4
        0x51
        0x7
        0xb
        0x0
        0x8
        0x62
        0x0
        0x8
        0x22
        0x0
        0x9
        0xa4
        0x0
        0x8
        0x2
        0x0
        0x8
        0x82
        0x0
        0x8
        0x42
        0x0
        0x9
        0xe4
        0x50
        0x7
        0x7
        0x0
        0x8
        0x5a
        0x0
        0x8
        0x1a
        0x0
        0x9
        0x94
        0x54
        0x7
        0x43
        0x0
        0x8
        0x7a
        0x0
        0x8
        0x3a
        0x0
        0x9
        0xd4
        0x52
        0x7
        0x13
        0x0
        0x8
        0x6a
        0x0
        0x8
        0x2a
        0x0
        0x9
        0xb4
        0x0
        0x8
        0xa
        0x0
        0x8
        0x8a
        0x0
        0x8
        0x4a
        0x0
        0x9
        0xf4
        0x50
        0x7
        0x5
        0x0
        0x8
        0x56
        0x0
        0x8
        0x16
        0xc0
        0x8
        0x0
        0x53
        0x7
        0x33
        0x0
        0x8
        0x76
        0x0
        0x8
        0x36
        0x0
        0x9
        0xcc
        0x51
        0x7
        0xf
        0x0
        0x8
        0x66
        0x0
        0x8
        0x26
        0x0
        0x9
        0xac
        0x0
        0x8
        0x6
        0x0
        0x8
        0x86
        0x0
        0x8
        0x46
        0x0
        0x9
        0xec
        0x50
        0x7
        0x9
        0x0
        0x8
        0x5e
        0x0
        0x8
        0x1e
        0x0
        0x9
        0x9c
        0x54
        0x7
        0x63
        0x0
        0x8
        0x7e
        0x0
        0x8
        0x3e
        0x0
        0x9
        0xdc
        0x52
        0x7
        0x1b
        0x0
        0x8
        0x6e
        0x0
        0x8
        0x2e
        0x0
        0x9
        0xbc
        0x0
        0x8
        0xe
        0x0
        0x8
        0x8e
        0x0
        0x8
        0x4e
        0x0
        0x9
        0xfc
        0x60
        0x7
        0x100
        0x0
        0x8
        0x51
        0x0
        0x8
        0x11
        0x55
        0x8
        0x83
        0x52
        0x7
        0x1f
        0x0
        0x8
        0x71
        0x0
        0x8
        0x31
        0x0
        0x9
        0xc2
        0x50
        0x7
        0xa
        0x0
        0x8
        0x61
        0x0
        0x8
        0x21
        0x0
        0x9
        0xa2
        0x0
        0x8
        0x1
        0x0
        0x8
        0x81
        0x0
        0x8
        0x41
        0x0
        0x9
        0xe2
        0x50
        0x7
        0x6
        0x0
        0x8
        0x59
        0x0
        0x8
        0x19
        0x0
        0x9
        0x92
        0x53
        0x7
        0x3b
        0x0
        0x8
        0x79
        0x0
        0x8
        0x39
        0x0
        0x9
        0xd2
        0x51
        0x7
        0x11
        0x0
        0x8
        0x69
        0x0
        0x8
        0x29
        0x0
        0x9
        0xb2
        0x0
        0x8
        0x9
        0x0
        0x8
        0x89
        0x0
        0x8
        0x49
        0x0
        0x9
        0xf2
        0x50
        0x7
        0x4
        0x0
        0x8
        0x55
        0x0
        0x8
        0x15
        0x50
        0x8
        0x102
        0x53
        0x7
        0x2b
        0x0
        0x8
        0x75
        0x0
        0x8
        0x35
        0x0
        0x9
        0xca
        0x51
        0x7
        0xd
        0x0
        0x8
        0x65
        0x0
        0x8
        0x25
        0x0
        0x9
        0xaa
        0x0
        0x8
        0x5
        0x0
        0x8
        0x85
        0x0
        0x8
        0x45
        0x0
        0x9
        0xea
        0x50
        0x7
        0x8
        0x0
        0x8
        0x5d
        0x0
        0x8
        0x1d
        0x0
        0x9
        0x9a
        0x54
        0x7
        0x53
        0x0
        0x8
        0x7d
        0x0
        0x8
        0x3d
        0x0
        0x9
        0xda
        0x52
        0x7
        0x17
        0x0
        0x8
        0x6d
        0x0
        0x8
        0x2d
        0x0
        0x9
        0xba
        0x0
        0x8
        0xd
        0x0
        0x8
        0x8d
        0x0
        0x8
        0x4d
        0x0
        0x9
        0xfa
        0x50
        0x7
        0x3
        0x0
        0x8
        0x53
        0x0
        0x8
        0x13
        0x55
        0x8
        0xc3
        0x53
        0x7
        0x23
        0x0
        0x8
        0x73
        0x0
        0x8
        0x33
        0x0
        0x9
        0xc6
        0x51
        0x7
        0xb
        0x0
        0x8
        0x63
        0x0
        0x8
        0x23
        0x0
        0x9
        0xa6
        0x0
        0x8
        0x3
        0x0
        0x8
        0x83
        0x0
        0x8
        0x43
        0x0
        0x9
        0xe6
        0x50
        0x7
        0x7
        0x0
        0x8
        0x5b
        0x0
        0x8
        0x1b
        0x0
        0x9
        0x96
        0x54
        0x7
        0x43
        0x0
        0x8
        0x7b
        0x0
        0x8
        0x3b
        0x0
        0x9
        0xd6
        0x52
        0x7
        0x13
        0x0
        0x8
        0x6b
        0x0
        0x8
        0x2b
        0x0
        0x9
        0xb6
        0x0
        0x8
        0xb
        0x0
        0x8
        0x8b
        0x0
        0x8
        0x4b
        0x0
        0x9
        0xf6
        0x50
        0x7
        0x5
        0x0
        0x8
        0x57
        0x0
        0x8
        0x17
        0xc0
        0x8
        0x0
        0x53
        0x7
        0x33
        0x0
        0x8
        0x77
        0x0
        0x8
        0x37
        0x0
        0x9
        0xce
        0x51
        0x7
        0xf
        0x0
        0x8
        0x67
        0x0
        0x8
        0x27
        0x0
        0x9
        0xae
        0x0
        0x8
        0x7
        0x0
        0x8
        0x87
        0x0
        0x8
        0x47
        0x0
        0x9
        0xee
        0x50
        0x7
        0x9
        0x0
        0x8
        0x5f
        0x0
        0x8
        0x1f
        0x0
        0x9
        0x9e
        0x54
        0x7
        0x63
        0x0
        0x8
        0x7f
        0x0
        0x8
        0x3f
        0x0
        0x9
        0xde
        0x52
        0x7
        0x1b
        0x0
        0x8
        0x6f
        0x0
        0x8
        0x2f
        0x0
        0x9
        0xbe
        0x0
        0x8
        0xf
        0x0
        0x8
        0x8f
        0x0
        0x8
        0x4f
        0x0
        0x9
        0xfe
        0x60
        0x7
        0x100
        0x0
        0x8
        0x50
        0x0
        0x8
        0x10
        0x54
        0x8
        0x73
        0x52
        0x7
        0x1f
        0x0
        0x8
        0x70
        0x0
        0x8
        0x30
        0x0
        0x9
        0xc1
        0x50
        0x7
        0xa
        0x0
        0x8
        0x60
        0x0
        0x8
        0x20
        0x0
        0x9
        0xa1
        0x0
        0x8
        0x0
        0x0
        0x8
        0x80
        0x0
        0x8
        0x40
        0x0
        0x9
        0xe1
        0x50
        0x7
        0x6
        0x0
        0x8
        0x58
        0x0
        0x8
        0x18
        0x0
        0x9
        0x91
        0x53
        0x7
        0x3b
        0x0
        0x8
        0x78
        0x0
        0x8
        0x38
        0x0
        0x9
        0xd1
        0x51
        0x7
        0x11
        0x0
        0x8
        0x68
        0x0
        0x8
        0x28
        0x0
        0x9
        0xb1
        0x0
        0x8
        0x8
        0x0
        0x8
        0x88
        0x0
        0x8
        0x48
        0x0
        0x9
        0xf1
        0x50
        0x7
        0x4
        0x0
        0x8
        0x54
        0x0
        0x8
        0x14
        0x55
        0x8
        0xe3
        0x53
        0x7
        0x2b
        0x0
        0x8
        0x74
        0x0
        0x8
        0x34
        0x0
        0x9
        0xc9
        0x51
        0x7
        0xd
        0x0
        0x8
        0x64
        0x0
        0x8
        0x24
        0x0
        0x9
        0xa9
        0x0
        0x8
        0x4
        0x0
        0x8
        0x84
        0x0
        0x8
        0x44
        0x0
        0x9
        0xe9
        0x50
        0x7
        0x8
        0x0
        0x8
        0x5c
        0x0
        0x8
        0x1c
        0x0
        0x9
        0x99
        0x54
        0x7
        0x53
        0x0
        0x8
        0x7c
        0x0
        0x8
        0x3c
        0x0
        0x9
        0xd9
        0x52
        0x7
        0x17
        0x0
        0x8
        0x6c
        0x0
        0x8
        0x2c
        0x0
        0x9
        0xb9
        0x0
        0x8
        0xc
        0x0
        0x8
        0x8c
        0x0
        0x8
        0x4c
        0x0
        0x9
        0xf9
        0x50
        0x7
        0x3
        0x0
        0x8
        0x52
        0x0
        0x8
        0x12
        0x55
        0x8
        0xa3
        0x53
        0x7
        0x23
        0x0
        0x8
        0x72
        0x0
        0x8
        0x32
        0x0
        0x9
        0xc5
        0x51
        0x7
        0xb
        0x0
        0x8
        0x62
        0x0
        0x8
        0x22
        0x0
        0x9
        0xa5
        0x0
        0x8
        0x2
        0x0
        0x8
        0x82
        0x0
        0x8
        0x42
        0x0
        0x9
        0xe5
        0x50
        0x7
        0x7
        0x0
        0x8
        0x5a
        0x0
        0x8
        0x1a
        0x0
        0x9
        0x95
        0x54
        0x7
        0x43
        0x0
        0x8
        0x7a
        0x0
        0x8
        0x3a
        0x0
        0x9
        0xd5
        0x52
        0x7
        0x13
        0x0
        0x8
        0x6a
        0x0
        0x8
        0x2a
        0x0
        0x9
        0xb5
        0x0
        0x8
        0xa
        0x0
        0x8
        0x8a
        0x0
        0x8
        0x4a
        0x0
        0x9
        0xf5
        0x50
        0x7
        0x5
        0x0
        0x8
        0x56
        0x0
        0x8
        0x16
        0xc0
        0x8
        0x0
        0x53
        0x7
        0x33
        0x0
        0x8
        0x76
        0x0
        0x8
        0x36
        0x0
        0x9
        0xcd
        0x51
        0x7
        0xf
        0x0
        0x8
        0x66
        0x0
        0x8
        0x26
        0x0
        0x9
        0xad
        0x0
        0x8
        0x6
        0x0
        0x8
        0x86
        0x0
        0x8
        0x46
        0x0
        0x9
        0xed
        0x50
        0x7
        0x9
        0x0
        0x8
        0x5e
        0x0
        0x8
        0x1e
        0x0
        0x9
        0x9d
        0x54
        0x7
        0x63
        0x0
        0x8
        0x7e
        0x0
        0x8
        0x3e
        0x0
        0x9
        0xdd
        0x52
        0x7
        0x1b
        0x0
        0x8
        0x6e
        0x0
        0x8
        0x2e
        0x0
        0x9
        0xbd
        0x0
        0x8
        0xe
        0x0
        0x8
        0x8e
        0x0
        0x8
        0x4e
        0x0
        0x9
        0xfd
        0x60
        0x7
        0x100
        0x0
        0x8
        0x51
        0x0
        0x8
        0x11
        0x55
        0x8
        0x83
        0x52
        0x7
        0x1f
        0x0
        0x8
        0x71
        0x0
        0x8
        0x31
        0x0
        0x9
        0xc3
        0x50
        0x7
        0xa
        0x0
        0x8
        0x61
        0x0
        0x8
        0x21
        0x0
        0x9
        0xa3
        0x0
        0x8
        0x1
        0x0
        0x8
        0x81
        0x0
        0x8
        0x41
        0x0
        0x9
        0xe3
        0x50
        0x7
        0x6
        0x0
        0x8
        0x59
        0x0
        0x8
        0x19
        0x0
        0x9
        0x93
        0x53
        0x7
        0x3b
        0x0
        0x8
        0x79
        0x0
        0x8
        0x39
        0x0
        0x9
        0xd3
        0x51
        0x7
        0x11
        0x0
        0x8
        0x69
        0x0
        0x8
        0x29
        0x0
        0x9
        0xb3
        0x0
        0x8
        0x9
        0x0
        0x8
        0x89
        0x0
        0x8
        0x49
        0x0
        0x9
        0xf3
        0x50
        0x7
        0x4
        0x0
        0x8
        0x55
        0x0
        0x8
        0x15
        0x50
        0x8
        0x102
        0x53
        0x7
        0x2b
        0x0
        0x8
        0x75
        0x0
        0x8
        0x35
        0x0
        0x9
        0xcb
        0x51
        0x7
        0xd
        0x0
        0x8
        0x65
        0x0
        0x8
        0x25
        0x0
        0x9
        0xab
        0x0
        0x8
        0x5
        0x0
        0x8
        0x85
        0x0
        0x8
        0x45
        0x0
        0x9
        0xeb
        0x50
        0x7
        0x8
        0x0
        0x8
        0x5d
        0x0
        0x8
        0x1d
        0x0
        0x9
        0x9b
        0x54
        0x7
        0x53
        0x0
        0x8
        0x7d
        0x0
        0x8
        0x3d
        0x0
        0x9
        0xdb
        0x52
        0x7
        0x17
        0x0
        0x8
        0x6d
        0x0
        0x8
        0x2d
        0x0
        0x9
        0xbb
        0x0
        0x8
        0xd
        0x0
        0x8
        0x8d
        0x0
        0x8
        0x4d
        0x0
        0x9
        0xfb
        0x50
        0x7
        0x3
        0x0
        0x8
        0x53
        0x0
        0x8
        0x13
        0x55
        0x8
        0xc3
        0x53
        0x7
        0x23
        0x0
        0x8
        0x73
        0x0
        0x8
        0x33
        0x0
        0x9
        0xc7
        0x51
        0x7
        0xb
        0x0
        0x8
        0x63
        0x0
        0x8
        0x23
        0x0
        0x9
        0xa7
        0x0
        0x8
        0x3
        0x0
        0x8
        0x83
        0x0
        0x8
        0x43
        0x0
        0x9
        0xe7
        0x50
        0x7
        0x7
        0x0
        0x8
        0x5b
        0x0
        0x8
        0x1b
        0x0
        0x9
        0x97
        0x54
        0x7
        0x43
        0x0
        0x8
        0x7b
        0x0
        0x8
        0x3b
        0x0
        0x9
        0xd7
        0x52
        0x7
        0x13
        0x0
        0x8
        0x6b
        0x0
        0x8
        0x2b
        0x0
        0x9
        0xb7
        0x0
        0x8
        0xb
        0x0
        0x8
        0x8b
        0x0
        0x8
        0x4b
        0x0
        0x9
        0xf7
        0x50
        0x7
        0x5
        0x0
        0x8
        0x57
        0x0
        0x8
        0x17
        0xc0
        0x8
        0x0
        0x53
        0x7
        0x33
        0x0
        0x8
        0x77
        0x0
        0x8
        0x37
        0x0
        0x9
        0xcf
        0x51
        0x7
        0xf
        0x0
        0x8
        0x67
        0x0
        0x8
        0x27
        0x0
        0x9
        0xaf
        0x0
        0x8
        0x7
        0x0
        0x8
        0x87
        0x0
        0x8
        0x47
        0x0
        0x9
        0xef
        0x50
        0x7
        0x9
        0x0
        0x8
        0x5f
        0x0
        0x8
        0x1f
        0x0
        0x9
        0x9f
        0x54
        0x7
        0x63
        0x0
        0x8
        0x7f
        0x0
        0x8
        0x3f
        0x0
        0x9
        0xdf
        0x52
        0x7
        0x1b
        0x0
        0x8
        0x6f
        0x0
        0x8
        0x2f
        0x0
        0x9
        0xbf
        0x0
        0x8
        0xf
        0x0
        0x8
        0x8f
        0x0
        0x8
        0x4f
        0x0
        0x9
        0xff
    .end array-data

    :array_c38
    .array-data 4
        0x50
        0x5
        0x1
        0x57
        0x5
        0x101
        0x53
        0x5
        0x11
        0x5b
        0x5
        0x1001
        0x51
        0x5
        0x5
        0x59
        0x5
        0x401
        0x55
        0x5
        0x41
        0x5d
        0x5
        0x4001
        0x50
        0x5
        0x3
        0x58
        0x5
        0x201
        0x54
        0x5
        0x21
        0x5c
        0x5
        0x2001
        0x52
        0x5
        0x9
        0x5a
        0x5
        0x801
        0x56
        0x5
        0x81
        0xc0
        0x5
        0x6001
        0x50
        0x5
        0x2
        0x57
        0x5
        0x181
        0x53
        0x5
        0x19
        0x5b
        0x5
        0x1801
        0x51
        0x5
        0x7
        0x59
        0x5
        0x601
        0x55
        0x5
        0x61
        0x5d
        0x5
        0x6001
        0x50
        0x5
        0x4
        0x58
        0x5
        0x301
        0x54
        0x5
        0x31
        0x5c
        0x5
        0x3001
        0x52
        0x5
        0xd
        0x5a
        0x5
        0xc01
        0x56
        0x5
        0xc1
        0xc0
        0x5
        0x6001
    .end array-data

    :array_cfc
    .array-data 4
        0x3
        0x4
        0x5
        0x6
        0x7
        0x8
        0x9
        0xa
        0xb
        0xd
        0xf
        0x11
        0x13
        0x17
        0x1b
        0x1f
        0x23
        0x2b
        0x33
        0x3b
        0x43
        0x53
        0x63
        0x73
        0x83
        0xa3
        0xc3
        0xe3
        0x102
        0x0
        0x0
    .end array-data

    :array_d3e
    .array-data 4
        0x0
        0x0
        0x0
        0x0
        0x0
        0x0
        0x0
        0x0
        0x1
        0x1
        0x1
        0x1
        0x2
        0x2
        0x2
        0x2
        0x3
        0x3
        0x3
        0x3
        0x4
        0x4
        0x4
        0x4
        0x5
        0x5
        0x5
        0x5
        0x0
        0x70
        0x70
    .end array-data

    :array_d80
    .array-data 4
        0x1
        0x2
        0x3
        0x4
        0x5
        0x7
        0x9
        0xd
        0x11
        0x19
        0x21
        0x31
        0x41
        0x61
        0x81
        0xc1
        0x101
        0x181
        0x201
        0x301
        0x401
        0x601
        0x801
        0xc01
        0x1001
        0x1801
        0x2001
        0x3001
        0x4001
        0x6001
    .end array-data

    :array_dc0
    .array-data 4
        0x0
        0x0
        0x0
        0x0
        0x1
        0x1
        0x2
        0x2
        0x3
        0x3
        0x4
        0x4
        0x5
        0x5
        0x6
        0x6
        0x7
        0x7
        0x8
        0x8
        0x9
        0x9
        0xa
        0xa
        0xb
        0xb
        0xc
        0xc
        0xd
        0xd
    .end array-data
.end method

.method constructor <init>()V
    .registers 1

    .line 50
    invoke-direct {p0}, Ljava/lang/Object;-><init>()V

    return-void
.end method

.method private huft_build([IIII[I[I[I[I[I[I[I)I
    .registers 44
    .param p1, "b"    # [I
    .param p2, "bindex"    # I
    .param p3, "n"    # I
    .param p4, "s"    # I
    .param p5, "d"    # [I
    .param p6, "e"    # [I
    .param p7, "t"    # [I
    .param p8, "m"    # [I
    .param p9, "hp"    # [I
    .param p10, "hn"    # [I
    .param p11, "v"    # [I

    .line 211
    move-object/from16 v0, p0

    move/from16 v3, p3

    move/from16 v4, p4

    move-object/from16 v6, p9

    const/4 v8, 0x0

    .line 212
    .local v8, "p":I
    move v9, v8

    move v8, v3

    .line 214
    .local v8, "i":I
    .local v9, "p":I
    :goto_b
    iget-object v10, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    add-int v11, p2, v9

    aget v11, p1, v11

    aget v12, v10, v11

    const/4 v13, 0x1

    add-int/2addr v12, v13

    aput v12, v10, v11

    .line 215
    add-int/2addr v9, v13

    .line 216
    const/4 v10, -0x1

    add-int/2addr v8, v10

    .line 217
    if-nez v8, :cond_278

    .line 219
    iget-object v11, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    const/4 v12, 0x0

    aget v11, v11, v12

    if-ne v11, v3, :cond_28

    .line 220
    aput v10, p7, v12

    .line 221
    aput v12, p8, v12

    .line 222
    return v12

    .line 226
    :cond_28
    aget v11, p8, v12

    .line 227
    .local v11, "l":I
    const/4 v14, 0x1

    .local v14, "j":I
    :goto_2b
    move v15, v14

    .end local v14    # "j":I
    .local v15, "j":I
    const/16 v10, 0xf

    move v13, v15

    .end local v15    # "j":I
    .local v13, "j":I
    if-gt v13, v10, :cond_3d

    .line 228
    iget-object v10, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    aget v10, v10, v13

    if-eqz v10, :cond_38

    .line 229
    goto :goto_3d

    .line 227
    :cond_38
    add-int/lit8 v14, v13, 0x1

    const/4 v10, -0x1

    const/4 v13, 0x1

    goto :goto_2b

    .line 232
    :cond_3d
    :goto_3d
    move v10, v13

    .line 233
    .local v10, "k":I
    if-ge v11, v13, :cond_41

    .line 234
    move v11, v13

    .line 236
    :cond_41
    const/16 v8, 0xf

    :goto_43
    if-eqz v8, :cond_50

    .line 237
    iget-object v12, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    aget v12, v12, v8

    if-eqz v12, :cond_4c

    .line 238
    goto :goto_50

    .line 236
    :cond_4c
    add-int/lit8 v8, v8, -0x1

    const/4 v12, 0x0

    goto :goto_43

    .line 241
    :cond_50
    :goto_50
    move v12, v8

    .line 242
    .local v12, "g":I
    if-le v11, v8, :cond_54

    .line 243
    move v11, v8

    .line 245
    :cond_54
    const/4 v14, 0x0

    aput v11, p8, v14

    .line 248
    const/4 v14, 0x1

    shl-int v15, v14, v13

    .local v15, "y":I
    :goto_5a
    move v14, v15

    .end local v15    # "y":I
    .local v14, "y":I
    const/4 v15, -0x3

    if-ge v13, v8, :cond_6d

    .line 249
    iget-object v5, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    aget v5, v5, v13

    sub-int v5, v14, v5

    move v14, v5

    if-gez v5, :cond_68

    .line 250
    return v15

    .line 248
    :cond_68
    add-int/lit8 v13, v13, 0x1

    shl-int/lit8 v15, v14, 0x1

    goto :goto_5a

    .line 253
    :cond_6d
    iget-object v5, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    aget v5, v5, v8

    sub-int v5, v14, v5

    move v14, v5

    if-gez v5, :cond_77

    .line 254
    return v15

    .line 256
    :cond_77
    iget-object v5, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    aget v19, v5, v8

    add-int v19, v19, v14

    aput v19, v5, v8

    .line 259
    iget-object v5, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->x:[I

    const/16 v18, 0x0

    move/from16 v13, v18

    const/16 v17, 0x1

    aput v18, v5, v17

    .line 260
    const/4 v5, 0x1

    .line 261
    .end local v9    # "p":I
    .local v5, "p":I
    const/16 v19, 0x2

    move v9, v5

    const/4 v5, 0x2

    .line 262
    .local v5, "xp":I
    .restart local v9    # "p":I
    :goto_8e
    const/16 v16, -0x1

    add-int/lit8 v8, v8, -0x1

    if-eqz v8, :cond_ab

    .line 263
    move/from16 v20, v8

    .end local v8    # "i":I
    .local v20, "i":I
    iget-object v8, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->x:[I

    move/from16 v21, v10

    .end local v10    # "k":I
    .local v21, "k":I
    iget-object v10, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    aget v10, v10, v9

    add-int/2addr v10, v13

    move v13, v10

    aput v10, v8, v5

    .line 264
    add-int/lit8 v5, v5, 0x1

    .line 265
    add-int/lit8 v9, v9, 0x1

    .line 261
    move/from16 v8, v20

    move/from16 v10, v21

    goto :goto_8e

    .line 269
    .end local v20    # "i":I
    .end local v21    # "k":I
    .restart local v8    # "i":I
    .restart local v10    # "k":I
    :cond_ab
    move/from16 v20, v8

    move/from16 v21, v10

    .end local v8    # "i":I
    .end local v10    # "k":I
    .restart local v20    # "i":I
    .restart local v21    # "k":I
    const/4 v8, 0x0

    .line 270
    .end local v20    # "i":I
    .restart local v8    # "i":I
    const/4 v9, 0x0

    .line 272
    :goto_b1
    add-int v10, p2, v9

    aget v10, p1, v10

    move v13, v10

    if-eqz v10, :cond_c2

    .line 273
    iget-object v10, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->x:[I

    aget v16, v10, v13

    add-int/lit8 v20, v16, 0x1

    aput v20, v10, v13

    aput v8, p11, v16

    .line 275
    :cond_c2
    add-int/lit8 v9, v9, 0x1

    .line 276
    add-int/lit8 v8, v8, 0x1

    if-lt v8, v3, :cond_274

    .line 277
    iget-object v10, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->x:[I

    aget v3, v10, v12

    .line 280
    .end local p3    # "n":I
    .local v3, "n":I
    iget-object v10, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->x:[I

    const/16 v16, 0x0

    move/from16 v8, v16

    aput v16, v10, v16

    .line 281
    const/4 v9, 0x0

    .line 282
    const/4 v10, -0x1

    .line 283
    .local v10, "h":I
    neg-int v1, v11

    .line 284
    .local v1, "w":I
    move/from16 v22, v1

    .end local v1    # "w":I
    .local v22, "w":I
    iget-object v1, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->u:[I

    aput v16, v1, v16

    .line 285
    const/4 v1, 0x0

    .line 286
    .local v1, "q":I
    move/from16 v16, v1

    move/from16 v20, v5

    move/from16 v5, v21

    const/4 v1, 0x0

    .line 289
    .end local v21    # "k":I
    .local v1, "z":I
    .local v5, "k":I
    .local v16, "q":I
    .local v20, "xp":I
    :goto_e5
    if-gt v5, v12, :cond_260

    .line 290
    move/from16 v23, v1

    .end local v1    # "z":I
    .local v23, "z":I
    iget-object v1, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    aget v1, v1, v5

    .line 291
    .local v1, "a":I
    :goto_ed
    add-int/lit8 v21, v1, -0x1

    .local v21, "a":I
    if-eqz v1, :cond_252

    .line 294
    .end local v1    # "a":I
    move/from16 v1, v23

    .end local v23    # "z":I
    .local v1, "z":I
    :goto_f3
    add-int v2, v22, v11

    move/from16 v24, v13

    .end local v13    # "j":I
    .local v24, "j":I
    if-le v5, v2, :cond_1b0

    .line 295
    add-int/lit8 v10, v10, 0x1

    .line 296
    add-int v22, v22, v11

    .line 298
    sub-int v1, v12, v22

    .line 299
    if-le v1, v11, :cond_103

    move v2, v11

    goto :goto_104

    :cond_103
    move v2, v1

    :goto_104
    move v1, v2

    .line 300
    sub-int v2, v5, v22

    move/from16 v25, v2

    const/16 v17, 0x1

    .end local v24    # "j":I
    .local v25, "j":I
    shl-int v2, v17, v2

    move/from16 v23, v2

    .local v23, "f":I
    add-int/lit8 v13, v21, 0x1

    if-le v2, v13, :cond_14f

    .line 302
    add-int/lit8 v2, v21, 0x1

    sub-int v23, v23, v2

    .line 303
    move v2, v5

    .line 304
    .end local v20    # "xp":I
    .local v2, "xp":I
    move/from16 v13, v25

    .end local v25    # "j":I
    .restart local v13    # "j":I
    if-ge v13, v1, :cond_14a

    .line 305
    :goto_11c
    const/16 v17, 0x1

    add-int/lit8 v13, v13, 0x1

    if-ge v13, v1, :cond_143

    .line 306
    move/from16 v26, v1

    .end local v1    # "z":I
    .local v26, "z":I
    shl-int/lit8 v1, v23, 0x1

    move/from16 v23, v1

    move/from16 v27, v13

    .end local v13    # "j":I
    .local v27, "j":I
    iget-object v13, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    add-int/lit8 v2, v2, 0x1

    aget v13, v13, v2

    if-gt v1, v13, :cond_138

    .line 307
    nop

    .line 313
    move/from16 v20, v2

    move/from16 v13, v27

    goto :goto_153

    .line 309
    :cond_138
    iget-object v1, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    aget v1, v1, v2

    sub-int v23, v23, v1

    .line 305
    move/from16 v1, v26

    move/from16 v13, v27

    goto :goto_11c

    .line 313
    .end local v26    # "z":I
    .end local v27    # "j":I
    .restart local v1    # "z":I
    .restart local v13    # "j":I
    :cond_143
    move/from16 v26, v1

    move/from16 v27, v13

    move/from16 v20, v2

    .end local v1    # "z":I
    .end local v13    # "j":I
    .restart local v26    # "z":I
    .restart local v27    # "j":I
    goto :goto_153

    .end local v26    # "z":I
    .end local v27    # "j":I
    .restart local v1    # "z":I
    .restart local v13    # "j":I
    :cond_14a
    move/from16 v26, v1

    move/from16 v20, v2

    .end local v1    # "z":I
    .restart local v26    # "z":I
    goto :goto_153

    .end local v2    # "xp":I
    .end local v13    # "j":I
    .end local v26    # "z":I
    .restart local v1    # "z":I
    .restart local v20    # "xp":I
    .restart local v25    # "j":I
    :cond_14f
    move/from16 v26, v1

    move/from16 v13, v25

    .end local v1    # "z":I
    .end local v25    # "j":I
    .restart local v13    # "j":I
    .restart local v26    # "z":I
    :goto_153
    const/4 v1, 0x1

    shl-int v2, v1, v13

    .line 316
    .end local v26    # "z":I
    .local v2, "z":I
    const/4 v1, 0x0

    aget v18, p10, v1

    add-int v1, v18, v2

    move/from16 v28, v12

    .end local v12    # "g":I
    .local v28, "g":I
    const/16 v12, 0x5a0

    if-le v1, v12, :cond_162

    .line 317
    return v15

    .line 319
    :cond_162
    iget-object v1, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->u:[I

    const/4 v12, 0x0

    aget v18, p10, v12

    move/from16 v16, v18

    aput v18, v1, v10

    .line 320
    aget v1, p10, v12

    add-int/2addr v1, v2

    aput v1, p10, v12

    .line 323
    if-eqz v10, :cond_1aa

    .line 324
    iget-object v1, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->x:[I

    aput v8, v1, v10

    .line 325
    iget-object v1, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    move/from16 v29, v2

    .end local v2    # "z":I
    .local v29, "z":I
    int-to-byte v2, v13

    aput v2, v1, v12

    .line 326
    iget-object v1, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    int-to-byte v2, v11

    const/4 v12, 0x1

    aput v2, v1, v12

    .line 327
    sub-int v1, v22, v11

    ushr-int v13, v8, v1

    .line 328
    iget-object v1, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    iget-object v2, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->u:[I

    add-int/lit8 v12, v10, -0x1

    aget v2, v2, v12

    sub-int v2, v16, v2

    sub-int/2addr v2, v13

    aput v2, v1, v19

    .line 329
    iget-object v1, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    iget-object v2, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->u:[I

    add-int/lit8 v12, v10, -0x1

    aget v2, v2, v12

    add-int/2addr v2, v13

    const/4 v12, 0x3

    mul-int/lit8 v2, v2, 0x3

    const/4 v7, 0x0

    invoke-static {v1, v7, v6, v2, v12}, Ljava/lang/System;->arraycopy(Ljava/lang/Object;ILjava/lang/Object;II)V

    .line 294
    .end local v23    # "f":I
    .end local v28    # "g":I
    .end local v29    # "z":I
    .restart local v1    # "z":I
    .restart local v12    # "g":I
    :goto_1a4
    move/from16 v12, v28

    move/from16 v1, v29

    goto/16 :goto_f3

    .line 331
    .end local v1    # "z":I
    .end local v12    # "g":I
    .restart local v2    # "z":I
    .restart local v23    # "f":I
    .restart local v28    # "g":I
    :cond_1aa
    move/from16 v29, v2

    const/4 v7, 0x0

    .end local v2    # "z":I
    .restart local v29    # "z":I
    aput v16, p7, v7

    goto :goto_1a4

    .line 336
    .end local v13    # "j":I
    .end local v23    # "f":I
    .end local v28    # "g":I
    .end local v29    # "z":I
    .restart local v1    # "z":I
    .restart local v12    # "g":I
    .restart local v24    # "j":I
    :cond_1b0
    move/from16 v28, v12

    .end local v12    # "g":I
    .restart local v28    # "g":I
    iget-object v2, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    sub-int v7, v5, v22

    int-to-byte v7, v7

    const/4 v12, 0x1

    aput v7, v2, v12

    .line 337
    if-lt v9, v3, :cond_1c4

    .line 338
    iget-object v2, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    const/16 v7, 0xc0

    const/4 v12, 0x0

    aput v7, v2, v12

    goto :goto_1fc

    .line 339
    :cond_1c4
    aget v2, p11, v9

    if-ge v2, v4, :cond_1e1

    .line 340
    iget-object v2, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    aget v7, p11, v9

    const/16 v12, 0x100

    if-ge v7, v12, :cond_1d2

    const/4 v12, 0x0

    goto :goto_1d4

    :cond_1d2
    const/16 v12, 0x60

    :goto_1d4
    int-to-byte v7, v12

    const/4 v12, 0x0

    aput v7, v2, v12

    .line 341
    iget-object v2, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    add-int/lit8 v7, v9, 0x1

    .local v7, "p":I
    aget v9, p11, v9

    .end local v9    # "p":I
    aput v9, v2, v19

    goto :goto_1fb

    .line 343
    .end local v7    # "p":I
    .restart local v9    # "p":I
    :cond_1e1
    iget-object v2, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    aget v7, p11, v9

    sub-int/2addr v7, v4

    aget v7, p6, v7

    add-int/lit8 v7, v7, 0x10

    add-int/lit8 v7, v7, 0x40

    int-to-byte v7, v7

    const/4 v13, 0x0

    aput v7, v2, v13

    .line 344
    iget-object v2, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    add-int/lit8 v7, v9, 0x1

    .restart local v7    # "p":I
    aget v9, p11, v9

    .end local v9    # "p":I
    sub-int/2addr v9, v4

    aget v9, p5, v9

    aput v9, v2, v19

    .line 348
    .end local v7    # "p":I
    .restart local v9    # "p":I
    :goto_1fb
    move v9, v7

    :goto_1fc
    sub-int v2, v5, v22

    const/4 v7, 0x1

    shl-int v2, v7, v2

    .line 349
    .local v2, "f":I
    ushr-int v7, v8, v22

    .end local v24    # "j":I
    .local v7, "j":I
    :goto_203
    if-ge v7, v1, :cond_21c

    .line 350
    iget-object v13, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    add-int v23, v16, v7

    move/from16 v30, v1

    move/from16 v31, v3

    const/4 v1, 0x3

    .end local v1    # "z":I
    .end local v3    # "n":I
    .local v30, "z":I
    .local v31, "n":I
    mul-int/lit8 v3, v23, 0x3

    const/4 v4, 0x0

    invoke-static {v13, v4, v6, v3, v1}, Ljava/lang/System;->arraycopy(Ljava/lang/Object;ILjava/lang/Object;II)V

    .line 349
    add-int/2addr v7, v2

    move/from16 v1, v30

    move/from16 v3, v31

    move/from16 v4, p4

    goto :goto_203

    .line 354
    .end local v30    # "z":I
    .end local v31    # "n":I
    .restart local v1    # "z":I
    .restart local v3    # "n":I
    :cond_21c
    move/from16 v30, v1

    move/from16 v31, v3

    const/4 v4, 0x0

    .end local v1    # "z":I
    .end local v3    # "n":I
    .restart local v30    # "z":I
    .restart local v31    # "n":I
    add-int/lit8 v1, v5, -0x1

    const/4 v3, 0x1

    shl-int v1, v3, v1

    move v13, v1

    .end local v7    # "j":I
    .restart local v13    # "j":I
    :goto_227
    and-int v1, v8, v13

    if-eqz v1, :cond_22f

    .line 355
    xor-int/2addr v8, v13

    .line 354
    ushr-int/lit8 v13, v13, 0x1

    goto :goto_227

    .line 357
    :cond_22f
    xor-int/2addr v8, v13

    .line 360
    const/4 v1, 0x1

    shl-int v3, v1, v22

    sub-int/2addr v3, v1

    .local v3, "mask":I
    :goto_234
    move v1, v3

    .line 361
    .end local v3    # "mask":I
    .local v1, "mask":I
    and-int v3, v8, v1

    iget-object v7, v0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->x:[I

    aget v7, v7, v10

    if-eq v3, v7, :cond_246

    .line 362
    add-int/lit8 v10, v10, -0x1

    .line 363
    sub-int v22, v22, v11

    .line 364
    const/4 v7, 0x1

    shl-int v3, v7, v22

    sub-int/2addr v3, v7

    goto :goto_234

    .line 290
    .end local v1    # "mask":I
    .end local v2    # "f":I
    :cond_246
    move/from16 v1, v21

    move/from16 v12, v28

    move/from16 v23, v30

    move/from16 v3, v31

    move/from16 v4, p4

    goto/16 :goto_ed

    .line 289
    .end local v28    # "g":I
    .end local v30    # "z":I
    .end local v31    # "n":I
    .local v3, "n":I
    .restart local v12    # "g":I
    .local v23, "z":I
    :cond_252
    move/from16 v31, v3

    move/from16 v28, v12

    const/4 v4, 0x0

    const/4 v7, 0x1

    .end local v3    # "n":I
    .end local v12    # "g":I
    .restart local v28    # "g":I
    .restart local v31    # "n":I
    add-int/lit8 v5, v5, 0x1

    .line 286
    move/from16 v1, v23

    move/from16 v4, p4

    goto/16 :goto_e5

    .line 369
    .end local v21    # "a":I
    .end local v23    # "z":I
    .end local v28    # "g":I
    .end local v31    # "n":I
    .local v1, "z":I
    .restart local v3    # "n":I
    .restart local v12    # "g":I
    :cond_260
    move/from16 v23, v1

    move/from16 v31, v3

    move/from16 v28, v12

    const/4 v4, 0x0

    const/4 v7, 0x1

    .end local v1    # "z":I
    .end local v3    # "n":I
    .end local v12    # "g":I
    .restart local v23    # "z":I
    .restart local v28    # "g":I
    .restart local v31    # "n":I
    if-eqz v14, :cond_271

    move/from16 v1, v28

    .end local v28    # "g":I
    .local v1, "g":I
    if-eq v1, v7, :cond_273

    const/4 v2, -0x5

    const/4 v4, -0x5

    goto :goto_273

    .end local v1    # "g":I
    .restart local v28    # "g":I
    :cond_271
    move/from16 v1, v28

    .end local v28    # "g":I
    .restart local v1    # "g":I
    :cond_273
    :goto_273
    return v4

    .line 272
    .end local v1    # "g":I
    .end local v10    # "h":I
    .end local v16    # "q":I
    .end local v20    # "xp":I
    .end local v22    # "w":I
    .end local v23    # "z":I
    .end local v31    # "n":I
    .local v5, "xp":I
    .restart local v12    # "g":I
    .local v21, "k":I
    .restart local p3    # "n":I
    :cond_274
    move/from16 v4, p4

    .end local v12    # "g":I
    .restart local v1    # "g":I
    goto/16 :goto_b1

    .line 212
    .end local v1    # "g":I
    .end local v5    # "xp":I
    .end local v11    # "l":I
    .end local v13    # "j":I
    .end local v14    # "y":I
    .end local v21    # "k":I
    :cond_278
    move/from16 v4, p4

    goto/16 :goto_b
.end method

.method static inflate_trees_fixed([I[I[[I[[I)I
    .registers 6
    .param p0, "bl"    # [I
    .param p1, "bd"    # [I
    .param p2, "tl"    # [[I
    .param p3, "td"    # [[I

    .line 443
    const/4 v0, 0x0

    const/16 v1, 0x9

    aput v1, p0, v0

    .line 444
    const/4 v1, 0x5

    aput v1, p1, v0

    .line 445
    sget-object v1, Lorg/jboss/netty/util/internal/jzlib/InfTree;->fixed_tl:[I

    aput-object v1, p2, v0

    .line 446
    sget-object v1, Lorg/jboss/netty/util/internal/jzlib/InfTree;->fixed_td:[I

    aput-object v1, p3, v0

    .line 447
    return v0
.end method

.method private initWorkArea(I)V
    .registers 8
    .param p1, "vsize"    # I

    .line 451
    iget-object v0, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->hn:[I

    const/16 v1, 0xf

    const/4 v2, 0x3

    const/16 v3, 0x10

    if-nez v0, :cond_23

    .line 452
    const/4 v0, 0x1

    new-array v0, v0, [I

    iput-object v0, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->hn:[I

    .line 453
    new-array v0, p1, [I

    iput-object v0, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->v:[I

    .line 454
    new-array v0, v3, [I

    iput-object v0, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    .line 455
    new-array v0, v2, [I

    iput-object v0, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    .line 456
    new-array v0, v1, [I

    iput-object v0, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->u:[I

    .line 457
    new-array v0, v3, [I

    iput-object v0, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->x:[I

    goto :goto_5a

    .line 459
    :cond_23
    iget-object v0, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->v:[I

    array-length v0, v0

    const/4 v4, 0x0

    if-ge v0, p1, :cond_2e

    .line 460
    new-array v0, p1, [I

    iput-object v0, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->v:[I

    goto :goto_38

    .line 462
    :cond_2e
    const/4 v0, 0x0

    .local v0, "i":I
    :goto_2f
    if-ge v0, p1, :cond_38

    .line 463
    iget-object v5, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->v:[I

    aput v4, v5, v0

    .line 462
    add-int/lit8 v0, v0, 0x1

    goto :goto_2f

    .line 466
    .end local v0    # "i":I
    :cond_38
    :goto_38
    const/4 v0, 0x0

    .restart local v0    # "i":I
    :goto_39
    if-ge v0, v3, :cond_42

    .line 467
    iget-object v5, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    aput v4, v5, v0

    .line 466
    add-int/lit8 v0, v0, 0x1

    goto :goto_39

    .line 469
    .end local v0    # "i":I
    :cond_42
    const/4 v0, 0x0

    .restart local v0    # "i":I
    :goto_43
    if-ge v0, v2, :cond_4c

    .line 470
    iget-object v5, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->r:[I

    aput v4, v5, v0

    .line 469
    add-int/lit8 v0, v0, 0x1

    goto :goto_43

    .line 473
    .end local v0    # "i":I
    :cond_4c
    iget-object v0, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    iget-object v2, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->u:[I

    invoke-static {v0, v4, v2, v4, v1}, Ljava/lang/System;->arraycopy(Ljava/lang/Object;ILjava/lang/Object;II)V

    .line 475
    iget-object v0, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->c:[I

    iget-object v1, p0, Lorg/jboss/netty/util/internal/jzlib/InfTree;->x:[I

    invoke-static {v0, v4, v1, v4, v3}, Ljava/lang/System;->arraycopy(Ljava/lang/Object;ILjava/lang/Object;II)V

    .line 477
    :goto_5a
    return-void
.end method


# virtual methods
.method inflate_trees_bits([I[I[I[ILorg/jboss/netty/util/internal/jzlib/ZStream;)I
    .registers 21
    .param p1, "c"    # [I
    .param p2, "bb"    # [I
    .param p3, "tb"    # [I
    .param p4, "hp"    # [I
    .param p5, "z"    # Lorg/jboss/netty/util/internal/jzlib/ZStream;

    .line 379
    move-object v12, p0

    move-object/from16 v13, p5

    const/16 v0, 0x13

    invoke-direct {p0, v0}, Lorg/jboss/netty/util/internal/jzlib/InfTree;->initWorkArea(I)V

    .line 380
    iget-object v0, v12, Lorg/jboss/netty/util/internal/jzlib/InfTree;->hn:[I

    const/4 v14, 0x0

    aput v14, v0, v14

    .line 381
    iget-object v10, v12, Lorg/jboss/netty/util/internal/jzlib/InfTree;->hn:[I

    iget-object v11, v12, Lorg/jboss/netty/util/internal/jzlib/InfTree;->v:[I

    const/4 v2, 0x0

    const/16 v3, 0x13

    const/16 v4, 0x13

    const/4 v5, 0x0

    const/4 v6, 0x0

    move-object v0, p0

    move-object/from16 v1, p1

    move-object/from16 v7, p3

    move-object/from16 v8, p2

    move-object/from16 v9, p4

    invoke-direct/range {v0 .. v11}, Lorg/jboss/netty/util/internal/jzlib/InfTree;->huft_build([IIII[I[I[I[I[I[I[I)I

    move-result v0

    .line 383
    .local v0, "result":I
    const/4 v1, -0x3

    if-ne v0, v1, :cond_2d

    .line 384
    const-string v1, "oversubscribed dynamic bit lengths tree"

    iput-object v1, v13, Lorg/jboss/netty/util/internal/jzlib/ZStream;->msg:Ljava/lang/String;

    goto :goto_39

    .line 385
    :cond_2d
    const/4 v1, -0x5

    if-eq v0, v1, :cond_34

    aget v2, p2, v14

    if-nez v2, :cond_39

    .line 386
    :cond_34
    const-string v2, "incomplete dynamic bit lengths tree"

    iput-object v2, v13, Lorg/jboss/netty/util/internal/jzlib/ZStream;->msg:Ljava/lang/String;

    .line 387
    const/4 v0, -0x3

    .line 389
    :cond_39
    :goto_39
    return v0
.end method

.method inflate_trees_dynamic(II[I[I[I[I[I[ILorg/jboss/netty/util/internal/jzlib/ZStream;)I
    .registers 27
    .param p1, "nl"    # I
    .param p2, "nd"    # I
    .param p3, "c"    # [I
    .param p4, "bl"    # [I
    .param p5, "bd"    # [I
    .param p6, "tl"    # [I
    .param p7, "td"    # [I
    .param p8, "hp"    # [I
    .param p9, "z"    # Lorg/jboss/netty/util/internal/jzlib/ZStream;

    .line 405
    move-object/from16 v12, p0

    move-object/from16 v13, p9

    const/16 v14, 0x120

    invoke-direct {v12, v14}, Lorg/jboss/netty/util/internal/jzlib/InfTree;->initWorkArea(I)V

    .line 406
    iget-object v0, v12, Lorg/jboss/netty/util/internal/jzlib/InfTree;->hn:[I

    const/4 v15, 0x0

    aput v15, v0, v15

    .line 407
    sget-object v5, Lorg/jboss/netty/util/internal/jzlib/InfTree;->cplens:[I

    sget-object v6, Lorg/jboss/netty/util/internal/jzlib/InfTree;->cplext:[I

    iget-object v10, v12, Lorg/jboss/netty/util/internal/jzlib/InfTree;->hn:[I

    iget-object v11, v12, Lorg/jboss/netty/util/internal/jzlib/InfTree;->v:[I

    const/4 v2, 0x0

    const/16 v4, 0x101

    move-object/from16 v0, p0

    move-object/from16 v1, p3

    move/from16 v3, p1

    move-object/from16 v7, p6

    move-object/from16 v8, p4

    move-object/from16 v9, p8

    invoke-direct/range {v0 .. v11}, Lorg/jboss/netty/util/internal/jzlib/InfTree;->huft_build([IIII[I[I[I[I[I[I[I)I

    move-result v11

    .line 408
    .local v11, "result":I
    const/4 v10, -0x4

    const/4 v9, -0x3

    if-nez v11, :cond_87

    aget v0, p4, v15

    if-nez v0, :cond_37

    .line 409
    move/from16 v3, p1

    move v14, v11

    const/4 v2, -0x3

    const/4 v4, -0x4

    goto :goto_8c

    .line 419
    :cond_37
    invoke-direct {v12, v14}, Lorg/jboss/netty/util/internal/jzlib/InfTree;->initWorkArea(I)V

    .line 420
    const/4 v4, 0x0

    sget-object v5, Lorg/jboss/netty/util/internal/jzlib/InfTree;->cpdist:[I

    sget-object v6, Lorg/jboss/netty/util/internal/jzlib/InfTree;->cpdext:[I

    iget-object v14, v12, Lorg/jboss/netty/util/internal/jzlib/InfTree;->hn:[I

    iget-object v8, v12, Lorg/jboss/netty/util/internal/jzlib/InfTree;->v:[I

    move-object/from16 v0, p0

    move-object/from16 v1, p3

    move/from16 v2, p1

    move/from16 v3, p2

    move-object/from16 v7, p7

    move-object/from16 v16, v8

    move-object/from16 v8, p5

    move-object/from16 v9, p8

    move-object v10, v14

    move v14, v11

    .end local v11    # "result":I
    .local v14, "result":I
    move-object/from16 v11, v16

    invoke-direct/range {v0 .. v11}, Lorg/jboss/netty/util/internal/jzlib/InfTree;->huft_build([IIII[I[I[I[I[I[I[I)I

    move-result v0

    .line 422
    .end local v14    # "result":I
    .local v0, "result":I
    if-nez v0, :cond_6b

    aget v2, p5, v15

    if-nez v2, :cond_68

    const/16 v2, 0x101

    move/from16 v3, p1

    if-le v3, v2, :cond_6a

    goto :goto_6d

    .line 435
    :cond_68
    move/from16 v3, p1

    :cond_6a
    return v15

    .line 423
    :cond_6b
    move/from16 v3, p1

    :goto_6d
    const/4 v2, -0x3

    if-ne v0, v2, :cond_75

    .line 424
    const-string v2, "oversubscribed distance tree"

    iput-object v2, v13, Lorg/jboss/netty/util/internal/jzlib/ZStream;->msg:Ljava/lang/String;

    goto :goto_86

    .line 425
    :cond_75
    const/4 v2, -0x5

    if-ne v0, v2, :cond_7e

    .line 426
    const-string v2, "incomplete distance tree"

    iput-object v2, v13, Lorg/jboss/netty/util/internal/jzlib/ZStream;->msg:Ljava/lang/String;

    .line 427
    const/4 v0, -0x3

    goto :goto_86

    .line 428
    :cond_7e
    const/4 v4, -0x4

    if-eq v0, v4, :cond_86

    .line 429
    const-string v2, "empty distance tree with lengths"

    iput-object v2, v13, Lorg/jboss/netty/util/internal/jzlib/ZStream;->msg:Ljava/lang/String;

    .line 430
    const/4 v0, -0x3

    .line 432
    :cond_86
    :goto_86
    return v0

    .line 409
    .end local v0    # "result":I
    .restart local v11    # "result":I
    :cond_87
    move/from16 v3, p1

    move v14, v11

    const/4 v2, -0x3

    const/4 v4, -0x4

    .end local v11    # "result":I
    .restart local v14    # "result":I
    :goto_8c
    if-ne v14, v2, :cond_93

    .line 410
    const-string v0, "oversubscribed literal/length tree"

    iput-object v0, v13, Lorg/jboss/netty/util/internal/jzlib/ZStream;->msg:Ljava/lang/String;

    goto :goto_9b

    .line 411
    :cond_93
    if-eq v14, v4, :cond_9b

    .line 412
    const-string v0, "incomplete literal/length tree"

    iput-object v0, v13, Lorg/jboss/netty/util/internal/jzlib/ZStream;->msg:Ljava/lang/String;

    .line 413
    const/4 v11, -0x3

    goto :goto_9c

    .line 415
    :cond_9b
    :goto_9b
    move v11, v14

    .end local v14    # "result":I
    .restart local v11    # "result":I
    :goto_9c
    return v11
.end method
