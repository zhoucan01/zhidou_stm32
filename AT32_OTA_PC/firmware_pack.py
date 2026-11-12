"""读取 OTA 镜像或把原始 APP 封装为 OTA 镜像。"""
import hashlib
import struct
HEADER_SIZE = 2048
MIN_RAW_SIZE = 8
MAX_RAW_SIZE = 55296
MAX_IMAGE_SIZE = HEADER_SIZE + MAX_RAW_SIZE

def inspect_image(image):
    if not HEADER_SIZE + MIN_RAW_SIZE <= len(image) <= MAX_IMAGE_SIZE:
        raise ValueError('请选择 OTA.bin（1KB版本区 + 1KB摘要区 + APP），不能使用完整镜像或原始APP')
    try:
        version=image[:1024].split(bytes([0]),1)[0].decode('utf-8')
    except UnicodeDecodeError:
        raise ValueError('版本头无效，请选择 OTA.bin') from None
    if not version or not version.isprintable():raise ValueError('缺少有效版本头')
    stack,entry=struct.unpack_from('<II',image,HEADER_SIZE)
    if not (0x20000000 < stack <= 0x20008000 and stack % 8 == 0 and entry & 1
            and 0x08004008 <= (entry & ~1) < 0x08004000+len(image)-HEADER_SIZE):
        raise ValueError('APP 向量或链接地址无效，请选择 OTA.bin')
    return {'version':version,'digest':image[1024:1056].hex(),'size':len(image)}


def set_version(image, version):
    inspect_image(image)
    encoded=version.encode('utf-8')
    if not encoded or len(encoded)>1024 or not version.isprintable() or chr(0) in version:
        raise ValueError('版本号不能为空，必须为可显示文本，UTF-8 编码不能超过1024字节')
    result=encoded.ljust(1024,bytes([0]))+image[1024:]
    return result,inspect_image(result)


def prepare_ota_image(source, version):
    """自动识别：封装镜像保留原摘要；原始APP计算摘要并添加2KB头。"""
    try:
        inspect_image(source)
    except ValueError:
        if not MIN_RAW_SIZE<=len(source)<=MAX_RAW_SIZE:
            raise ValueError('请选择原始 APP codeBin.bin（8..55296字节）或 OTA.bin；不能使用直烧镜像或HEX') from None
        stack,entry=struct.unpack_from('<II',source)
        if not (0x20000000<stack<=0x20008000 and stack%8==0 and entry&1
                and 0x08004008<=(entry&~1)<0x08004000+len(source)):
            raise ValueError('原始 APP 向量无效，必须链接到0x08004000；也可选择有效的 OTA.bin') from None
        digest=hashlib.sha256(source).digest()
        template=b'raw-app'.ljust(1024,b'\0')+digest.ljust(1024,b'\0')+source
        image,metadata=set_version(template,version)
        metadata['source_format']='raw_app'
    else:
        image,metadata=source,inspect_image(source)
        metadata['source_format']='ota'
    return image,metadata

OTA_FILENAME = "OTA.bin"
RAW_APP_FILENAME = "codeBin.bin"
FACTORY_FILENAME = "factory.bin"
