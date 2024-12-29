# Writen by Wang Ziyu @ 2024-12-21

# coding=utf-8
import os
import requests
from apig_sdk import signer

if __name__ == '__main__':
    sig = signer.Signer()
    # Set the AK/SK to sign and authenticate the request.
    # 认证用的ak和sk硬编码到代码中或者明文存储都有很大的安全风险，建议在配置文件或者环境变量中密文存放，使用时解密，确保安全；
    # 本示例以ak和sk保存在环境变量中为例，运行本示例前请先在本地环境中设置环境变量HUAWEICLOUD_SDK_AK和HUAWEICLOUD_SDK_SK。
    sig.Key = os.getenv('HUAWEICLOUD_SDK_AK')
    sig.Secret = os.getenv('HUAWEICLOUD_SDK_SK')

    print("sig.Key: ", sig.Key)

    # The following example shows how to set the request URL and parameters to query a VPC list.
    # Set request Endpoint.
    # Specify a request method, such as GET, PUT, POST, DELETE, HEAD, and PATCH.
    # Set request URI.
    # Set parameters for the request URL.
    r = signer.HttpRequest(
        "POST", "https://e91356e3c17949bbbb3b8231d6fe5e37.apig.cn-north-4.huaweicloudapis.com/v1/infers/c2568cca-7f66-46a3-8189-d203482bbf67")
    # Add header parameters, for example, x-domain-id for invoking a global service and x-project-id for invoking a project-level service.
    r.headers = {"content-type": "application/json",
                 "x-sdk-content-sha256": "UNSIGNED-PAYLOAD"}
    # Add a body if you have specified the PUT or POST method. Special characters, such as the double quotation mark ("), contained in the body must be escaped.
    r.body = "./assets/test.jpg"
    files = {
        'image': open(r.body, 'rb')
    }
    sig.Sign(r)
    print(r.headers["X-Sdk-Date"])
    print(r.headers["Authorization"])
    print("r.method: ", r.method)
    print("r.scheme: ", r.scheme)
    print("r.host: ", r.host)
    print("r.uri: ", r.uri)
    print("r.query: ", r.query)
    print("r.headers: ", r.headers)
    resp = requests.post(r.scheme + "://" +
                         r.host + r.uri, headers=r.headers, files=files)
    print(resp.status_code, resp.reason)
    print(resp.content)
