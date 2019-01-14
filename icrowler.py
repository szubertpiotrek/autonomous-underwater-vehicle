//żeby zainstalowac  -> pip install icrawler
from icrawler.builtin import GoogleImageCrawler
for keyword in ['underwater ball']:  //ustalasz czego szukasz
    google_crawler = GoogleImageCrawler(
        parser_threads=2,
        downloader_threads=4,
        storage={'root_dir': 'images/{}'.format(keyword)}  //do jakiego folderu ma pobrac
   
    )
    google_crawler.crawl(
        keyword=keyword, max_num=10, min_size=(200, 200))  //max_num -> ile tych zdjec
